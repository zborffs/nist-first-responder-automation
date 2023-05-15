/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * 
 * from https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "utils.hpp"

/// <global variables>
// these variables contain the incoming pose data of the drone and the april tag (must be global so they can be used with callbacks)
geometry_msgs::PoseStamped geo_msg_pose_stamped_apriltag;
geometry_msgs::PoseStamped geo_msg_pose_stamped_drone;
bool geo_msg_pose_stamped_drone_data_in = 0;
bool geo_msg_pose_stamped_apriltag_data_in = 0;
bool all_in = 0;

// this variable needs to be global, because, ...
unsigned previous_at_in_seq{0}; // rename later...

// this variable needs to be global, because...
bool have_seen_apriltag_at_least_once{false};

// this variable needs to be global, because 

// </global variables>

void at_cb(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_apriltag_ptr){
   geo_msg_pose_stamped_apriltag = *geo_msgs_pose_stamped_apriltag_ptr;
   geo_msg_pose_stamped_apriltag_data_in = 1;  // set to true, since we must have received the data to be in this function

}

void geo_msg_pose_stamped_drone_callback(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_drone_ptr) {
    geo_msg_pose_stamped_drone = *geo_msgs_pose_stamped_drone_ptr;
    geo_msg_pose_stamped_drone_data_in = 1;  // set to true, since we must have received the data to be in this function
}


int main(int argc, char **argv) {
    // boilerplate code for node
    ros::init(argc, argv, "generate_waypoints_node"); // name the ROS node
    ros::NodeHandle nh;
    
    // declare the publishers and subscribers
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 5);
    ros::Subscriber apriltag_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose", 1, at_cb);
    ros::Subscriber local_info_sub = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, geo_msg_pose_stamped_drone_callback);
    ros::Publisher desired_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/desired_position", 10);
    ros::Publisher apriltag_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);

    // current bucket to wait at...
    Bucket bucket = Bucket::Bucket1;

    // Declare relevant poses
    Eigen::Matrix4d H_body_apriltag; // This is the pose of the apriltag in the body frame
    Eigen::Matrix4d H_inertial_apriltag; // this is the pose of the apriltag in the inertial frame
    Eigen::Matrix4d H_inertial_offset; // this is the pose of the desired position of the drone in the inertial frame

    // Set the publishing rate of THIS node... the setpoint publishing rate MUST be faster than 2Hz (for mavros)
    ros::Rate rate(20.0);

    // define the static fields of the "PositionTarget" struct
    mavros_msgs::PositionTarget desired_pose_inertial_body;
    desired_pose_inertial_body.coordinate_frame = 1; // this must be set to '1' for ineritial control
	desired_pose_inertial_body.type_mask = desired_pose_inertial_body.IGNORE_VX | desired_pose_inertial_body.IGNORE_VY | desired_pose_inertial_body.IGNORE_VZ | desired_pose_inertial_body.IGNORE_AFZ | desired_pose_inertial_body.IGNORE_AFY | desired_pose_inertial_body.IGNORE_AFX;

    // define the count (used for seeing whether we are currently / have ever seen an apriltag)
    int count = 0;
    int number_of_iterations_at_bucket_i{0};
    while (ros::ok()) {

        if (previous_at_in_seq != geo_msg_pose_stamped_apriltag.header.seq) {
            previous_at_in_seq = (unsigned)geo_msg_pose_stamped_apriltag.header.seq;
            count = 0;
        } else {
            ++count;
        }

        if (count == 20) {
            // recall rate is 20 Hz, therefore, 20 cycles is equivalent to 20 / 20 = 1 seconds
            geo_msg_pose_stamped_apriltag_data_in = 0;
        }
  
        // document me...
        if (geo_msg_pose_stamped_apriltag_data_in && geo_msg_pose_stamped_drone_data_in) {
            all_in = 1;
        } else {
            all_in = 0;
        }

        // Convert the pose of the drone in the inertial frame from a geometry_msgs::PoseStamped object
        // into a homogeneous tranform (as represented by a 4x4 Eigen matrix)
        Eigen::Matrix4d H_inertial_body = geo_msg_pose_stamped_to_homogeneous_tf(geo_msg_pose_stamped_drone); // [vector in inertial frame] = H_inertial_body [vector in body frame]; also represents the position of the drone in the inertial frame

        // Convert the pose of the apriltag in the camera frame from a geometry_msgs::PoseStamped object
        // into a homogeneous tranform (as represented by a 4x4 Eigen matrix)
        Eigen::Matrix4d H_camera_apriltag = geo_msg_pose_stamped_to_homogeneous_tf(geo_msg_pose_stamped_apriltag);

        // Define a static homogeneous transformation called "H_camera_body" that exhibits zero translation converts
        // transforms 4-vectors in the body frame to 4-vectors in the camera frame.
        // [vector expressed in camera frame] = H_camera_body * [vector expressed in body frame]
        // this was generated by looking at ModelAI's documentation for how the Camera frame and body frames are defined. They are that:
        // - Camera is pointing 45 deg down looking forward. = tracking camera
        Eigen::Matrix4d H_camera_body; // formerly H_M_B
        H_camera_body << 0,-1,0,0,-0.707,0,-0.707, 0, 0.707, 0, -0.707, 0, 0, 0, 0, 1;        

        if (all_in) {
            // we must have received an apriltag measurement and a drone position measurement in this iteration...
            // let's save the apriltag position in the inertial frame to a global variable.
            have_seen_apriltag_at_least_once = true;

            // Determine the pose of the apriltag in the body frame
            H_body_apriltag = H_camera_body.inverse() * H_camera_apriltag; // 

            // Determine the pose of the april tag in the local inertial frame
            H_inertial_apriltag = H_inertial_body * H_body_apriltag; // good

            // Determine the desired pose of the drone so that it has the desired offset between itself and the apriltag
            // Eigen::Matrix4d H_offset_apriltag = get_offset_to_apriltag(bucket);
            // H_inertial_offset = H_inertial_body * H_body_apriltag * H_offset_apriltag.inverse();

            // Print the pose of the apriltag in the body-frame
            ROS_INFO_STREAM("Currently seeing apriltag"); // print out position?
        } else if (!have_seen_apriltag_at_least_once) {
            // if we reached this point, that means we have never seen the april tag!
            ROS_INFO_STREAM("Don't know where apriltag is");
        } else {
            // if we reached this point, that means, we are not seeing the apriltag right now AND we have seen the 
            // apriltag in the past.

            // Print the pose of the apriltag in the body-frame
            ROS_INFO_STREAM("Last known position of apriltag:"); // print out position?
        }

        // Determine the desired pose of the drone so that it has the desired offset between itself and the apriltag
        Eigen::Matrix4d H_offset_apriltag = get_offset_to_apriltag(bucket);
        H_inertial_offset = H_inertial_apriltag * H_offset_apriltag.inverse();
        // ROS_INFO_STREAM("H_inertial_offset: \n" << homogeneous_tf_to_string(H_inertial_offset));
        // H_inertial_offset = H_inertial_body * H_body_apriltag * H_offset_apriltag.inverse();

        std::string bucket_string = bucket_to_string(bucket);
        ROS_INFO_STREAM("Currently on bucket " << bucket_string);

        // Publish the pose of the apriltag in the inertial ("map") frame to "/tag_detections/tagpose_inertial"
        geometry_msgs::PoseStamped apriltag_inertial_pub_data = homogeneous_tf_to_geo_msg_pose_stamped(H_inertial_apriltag);
        apriltag_inertial_pub_data.header.frame_id = "map";
        apriltag_inertial_pub_data.header.stamp = geo_msg_pose_stamped_drone.header.stamp;
        apriltag_pos_pub.publish(apriltag_inertial_pub_data);

        // Publish the desired pose of the drone in the inertial ("map") frame to "/desired_position"
        geometry_msgs::PoseStamped desired_inertial_pub_data = homogeneous_tf_to_geo_msg_pose_stamped(H_inertial_offset);
        desired_inertial_pub_data.header.frame_id = "map";
        desired_inertial_pub_data.header.stamp = geo_msg_pose_stamped_drone.header.stamp;
        desired_pos_pub.publish(desired_inertial_pub_data);

        desired_pose_inertial_body.header.stamp = ros::Time::now();
        desired_pose_inertial_body.position.x = H_inertial_offset(0,3);
        desired_pose_inertial_body.position.y = H_inertial_offset(1,3);
        desired_pose_inertial_body.position.z = H_inertial_offset(2,3);
        Eigen::Vector3d desired_euler_angles_3_2_1 = H_inertial_offset.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
        Eigen::Vector3d desired_euler_angles_3_1_2 = H_inertial_offset.topLeftCorner<3, 3>().eulerAngles(2, 0, 1); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
        if (std::abs(desired_euler_angles_3_1_2[0]) > 3.14 / 2.0) {
            desired_euler_angles_3_2_1[0] += 3.1415;
        }
        desired_pose_inertial_body.yaw = desired_euler_angles_3_2_1[0] - 3.14; // think this will work?
        local_pos_pub_mavros.publish(desired_pose_inertial_body);

        // Eigen::Vector3d actual_euler_angles_1 = H_inertial_body.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
        // Eigen::Vector3d actual_euler_angles_2 = H_inertial_body.topLeftCorner<3, 3>().eulerAngles(2, 0, 1); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
        // Eigen::Vector3d actual_euler_angles = H_inertial_body.topLeftCorner<3, 3>().eulerAngles(0,2,1); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis

        if (drone_is_approximately_at_offset(H_inertial_body, H_inertial_offset, 3.14 / 180 * 20, 0.20)) {
            // tolerance is 20 degrees and 20 centimeters
            number_of_iterations_at_bucket_i++;
        }

        if (number_of_iterations_at_bucket_i == 8 * 20) {
            // the update frequency of this node is approximately 20 hz...
            // if we have been waiting at the bucket for 8 seconds, that corresponds to 5 * 20 = 100 iterations, then switch to the
            // next bucket
            number_of_iterations_at_bucket_i = 0;

            switch (bucket) {
                case Bucket1:
                    bucket = Bucket::Bucket1a;
                    break;
                case Bucket1a:
                    bucket = Bucket::Bucket2;
                    break;
                case Bucket2:
                    bucket = Bucket::Bucket2a;
                    break;
                case Bucket2a:
                    bucket = Bucket::Bucket3;
                    break;
                case Bucket3:
                    bucket = Bucket::Bucket3a;
                    break;
                case Bucket3a:
                    bucket = Bucket::Bucket4;
                    break;
                case Bucket4:
                    bucket = Bucket::Bucket4a;
                    break;
                case Bucket4a:
                    bucket = Bucket::Bucket1;
                    break;
                

                ROS_INFO_STREAM("NICE, MOVE TO NEXT BUCKET!");
            }
        }

        // ROS_INFO_STREAM("ACTUAL ROLL} " << actual_euler_angles[0]);
        
        // ROS_INFO_STREAM("ACTUAL YAW (3-2-1) " << actual_euler_angles_1[0]);
        // ROS_INFO_STREAM("ACTUAL YAW (1-2-3) " << actual_euler_angles_2[2]);
        // ROS_INFO_STREAM("DESIRED YAW " << desired_euler_angles[0]);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
