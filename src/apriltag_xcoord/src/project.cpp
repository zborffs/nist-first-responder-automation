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

std::string homogeneous_tf_to_string(const Eigen::Matrix4d& H) {
    std::string ret("");
    ret += "[";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (j == 3 && i == 3) {
                ret += std::to_string(H(i,j)) + "]\n";
            } else {
                ret += std::to_string(H(i,j)) + ", ";
            }
            
        }
        ret += "\n"; //idk
    }
    return ret;
}

void at_cb(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_apriltag_ptr){
   geo_msg_pose_stamped_apriltag = *geo_msgs_pose_stamped_apriltag_ptr;
   geo_msg_pose_stamped_apriltag_data_in = 1;  // set to true, since we must have received the data to be in this function

}

void geo_msg_pose_stamped_drone_callback(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_drone_ptr) {
    geo_msg_pose_stamped_drone = *geo_msgs_pose_stamped_drone_ptr;
    geo_msg_pose_stamped_drone_data_in = 1;  // set to true, since we must have received the data to be in this function
}

/**
 * converts a homogeneous transformation (represented by a 4x4 matrix in Eigen) into a geometry_msgs::PoseStamped
 * object. 
 * inputs: 
 * - a homogeneous transformation
 * outputs:
 * - a geometry_msgs::PoseStamped object representing the same pose as was represented in the homogeneous transformation
 * 
 * Example:
 * ```c++
 * Eigen::Matrix4d H_inertial_body;
 * ...
 * geometry_msgs::PoseStamped my_pose_stamped = homogeneous_tf_to_geo_msg_pose_stamped(H_inertial_body);
 * ...
 * ```
 * so in this example the pose of the "body" (i.e. the drone) is represented in the inertial frame is encapsulated in a
 * 4x4 Matrix in Eigen. This is transformed into a geometry_msgs:PoseStamped object representing the pose of the drone
 * again in the inertial frame. 
 * 
 * Notes:
 * The homogeneous transform holds orientation information in top left 3x3 matrix in the form of a rotationa matrix. 
 * The geometry_msgs::PoseStamped object holds orientation information in a quaternion
 * The homogeneous transform holds position information in the top right 3x1 vector. 
 * The geometry_msgs::PoseStamped object holds orientation information in the same 3x1 vector.
 */
geometry_msgs::PoseStamped homogeneous_tf_to_geo_msg_pose_stamped(const Eigen::Matrix4d& homogeneous_tf) {
    // declare return object
    geometry_msgs::PoseStamped ret;

    // convert the top 3x3 rotation matrix into quaternion
    Eigen::Quaterniond quat(homogeneous_tf.topLeftCorner<3, 3>());

    // populate entries of PoseStamped object corresponding to orientation with quaternion data
    ret.pose.orientation.w = quat.w();
    ret.pose.orientation.x = quat.x();
    ret.pose.orientation.y = quat.y();
    ret.pose.orientation.z = quat.z();

    // populate entries of the PoseStamped object corresponding to position with homogeneous tf's positional data
    ret.pose.position.x = homogeneous_tf(0,3);
    ret.pose.position.y = homogeneous_tf(1,3);
    ret.pose.position.z = homogeneous_tf(2,3);

    return ret;
}

/**
 * pos_aprtiletag_inertial is a 4x1 vector. The first 3 elements of which are the position of the april tag in the inertial frame
 * pos_desiredoffset_inertial is a 4x1 vector. The first 3 elements of which are the desired position between the drone and the april tag in the inertial frame
 */
Eigen::Vector4d control_algorithm(const Eigen::Vector4d pos_aprtiltag_inertial, const Eigen::Vector4d pos_desiredoffset_inertial) {
    return pos_aprtiltag_inertial - pos_desiredoffset_inertial; // this is a minus sign, because, walk from origin of frame to april tag, then walk from april tag to the offset position. but the offset position (i think) is expressed from
}


int main(int argc, char **argv) {
    // assign these to zero why?
    geo_msg_pose_stamped_apriltag.pose.position.x = 0;
    geo_msg_pose_stamped_apriltag.pose.position.y = 0;
    geo_msg_pose_stamped_apriltag.pose.position.z = 0;


    // boilerplate code for node
    ros::init(argc, argv, "generate_waypoints_node"); // name the ROS node
    ros::NodeHandle nh;
    
    // .///
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    ros::Subscriber apriltag_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose", 1, at_cb);
    ros::Subscriber local_info_sub = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, geo_msg_pose_stamped_drone_callback);
    ros::Publisher desired_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/desired_position", 10);
    ros::Publisher apriltag_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);

    // This is the pose of the apriltag in the body frame
    Eigen::Matrix4d H_body_apriltag;

    // this is the pose of the apriltag in the inertial frame
    Eigen::Matrix4d H_inertial_apriltag;

    // this is the pose of the desired position of the drone in the inertial frame
    Eigen::Matrix4d H_inertial_offset;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    mavros_msgs::PositionTarget desired_pose_inertial_body;
    desired_pose_inertial_body.coordinate_frame = 1; // this must be set to '1' for ineritial control
    // desired_pose_inertial_body.yaw = 0; // pi/2 is facing opposite wall as tv; 0 faces into the building 
	desired_pose_inertial_body.type_mask = desired_pose_inertial_body.IGNORE_VX | desired_pose_inertial_body.IGNORE_VY | desired_pose_inertial_body.IGNORE_VZ | desired_pose_inertial_body.IGNORE_AFZ | desired_pose_inertial_body.IGNORE_AFY | desired_pose_inertial_body.IGNORE_AFX;

    ros::Time last_request = ros::Time::now();

    int count = 0;
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

        // Extract the position of the drone in the local inertial frame. 
        // Recall: that when starting the vehicle indoors, the the y-axis of the local inertial frame is defined as the direction
        // the drone is facing upon starting up!
        // Recall: that the body frame of the drone is an Forward-left-up (FLU) frame:
        // - x-axis points out the front of the drone
        // - y-axis points out the 'left' side of the drone
        // - z-axis points out the top of the drone
        double x_inertial_body = geo_msg_pose_stamped_drone.pose.position.x;
        double y_inertial_body = geo_msg_pose_stamped_drone.pose.position.y;
        double z_inertial_body = geo_msg_pose_stamped_drone.pose.position.z;

        // Extract the orientation of the drone in the local inertial frame. Express this orientation as a rotation matrix
        Eigen::Quaterniond quaternion_body_inertial(geo_msg_pose_stamped_drone.pose.orientation.w, geo_msg_pose_stamped_drone.pose.orientation.x, geo_msg_pose_stamped_drone.pose.orientation.y, geo_msg_pose_stamped_drone.pose.orientation.z);
        Eigen::Matrix3d R_inertial_body = quaternion_body_inertial.toRotationMatrix();

        // Populate a homogeneous tranformtation representing the pose of the drone in the local inertial frame using the 
        // data we just extracted!
        Eigen::Matrix4d H_inertial_body; // [vector in inertial frame] = H_inertial_body [vector in body frame]; also represents the position of the drone in the inertial frame
        H_inertial_body.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
        H_inertial_body.block(0,0,3,3) = R_inertial_body;
        H_inertial_body(3,3) = 1.0;
        H_inertial_body(0,3) = x_inertial_body;
        H_inertial_body(1,3) = y_inertial_body;
        H_inertial_body(2,3) = z_inertial_body;
        Eigen::Vector4d pos_inertial_body;
        pos_inertial_body << H_inertial_body(0,3), H_inertial_body(1,3), H_inertial_body(2,3), 1;

        // Extract the position of the apriltag in the camera frame. Recall:
        // - Z is coincident with optical axis; 
        // - Y is "down" in camera frame; 
        // - X is "to-the-right" when looking from vehicle out
        double x_camera_apriltag = geo_msg_pose_stamped_apriltag.pose.position.x;
        double y_camera_apriltag = geo_msg_pose_stamped_apriltag.pose.position.y;
        double z_camera_apriltag = geo_msg_pose_stamped_apriltag.pose.position.z;

        // Extract the orientation of the april tag frame w.r.t. the camera frame. Express this orientation as a rotation matrix
        Eigen::Quaterniond quat_camera_apriltag(geo_msg_pose_stamped_apriltag.pose.orientation.w, geo_msg_pose_stamped_apriltag.pose.orientation.x, geo_msg_pose_stamped_apriltag.pose.orientation.y, geo_msg_pose_stamped_apriltag.pose.orientation.z); // recall that Eigen's convention for quaternions is to have the 'w' scalar first
        Eigen::Matrix3d R_camera_apriltag = quat_camera_apriltag.toRotationMatrix();
        
        // Create a homogeneous transformation (representing the pose of the Apriltag in the Camera frame) with the position 
        // and orientation information we just extracted
        Eigen::Matrix4d H_camera_apriltag;
        H_camera_apriltag.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
        H_camera_apriltag.block(0,0,3,3) = R_camera_apriltag;
        H_camera_apriltag(3,3) = 1.0;
        H_camera_apriltag(0,3) = x_camera_apriltag;
        H_camera_apriltag(1,3) = y_camera_apriltag;
        H_camera_apriltag(2,3) = z_camera_apriltag;

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
            Eigen::Matrix4d H_apriltag_offset;
            H_apriltag_offset << -0.999663, 0.031221, 0.000515, 0.550040, 0.031171, 0.998714, -0.039981, -0.516124, -0.001763, -0.039951, -0.999351, -0.407456, 0.000000, 0.000000, 0.000000, 1.000000; // associated with facing bucket 1, this is the position of the apriltag in the body frame.
            H_inertial_offset = H_inertial_body * H_body_apriltag * H_apriltag_offset.inverse(); //.inverse();

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

        // desired_pose_inertial_body.header.stamp = ros::Time::now();
        // desired_pose_inertial_body.position.x = u(0);
        // desired_pose_inertial_body.position.y = u(1);
        // desired_pose_inertial_body.position.y = u(2);
        // desired_pose_inertial_body.position.z = 0.7;
        // local_pos_pub_mavros.publish(desired_pose_inertial_body);

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
        Eigen::Vector3d desired_euler_angles = H_inertial_offset.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
        desired_pose_inertial_body.yaw = desired_euler_angles[0]; // think this will work?

        // Eigen::Vector3d actual_euler_angles = H_inertial_body.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
        // ROS_INFO_STREAM("DESIRED YAW: " << std::to_string(desired_euler_angles[0]));
        // ROS_INFO_STREAM("ACTUAL YAW: " << std::to_string(actual_euler_angles[0]));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
