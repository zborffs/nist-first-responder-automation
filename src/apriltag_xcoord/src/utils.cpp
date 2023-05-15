#include "utils.hpp"


/**
 * represents the homogeneous transform as a string for readibility
*/
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
 * converts a geometry_msgs::PoseStamped into a homogeneous tranformation
 */
Eigen::Matrix4d geo_msg_pose_stamped_to_homogeneous_tf(geometry_msgs::PoseStamped& msg) {
    Eigen::Matrix4d homogeneous_tf;

    // Extract the orientation of the drone in the local inertial frame. Express this orientation as a rotation matrix
    Eigen::Quaterniond quat(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Populate a homogeneous tranformtation representing the pose of the drone in the local inertial frame using the 
    // data we just extracted!
    homogeneous_tf.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
    homogeneous_tf.block(0,0,3,3) = rotation_matrix;
    homogeneous_tf(3,3) = 1.0;
    homogeneous_tf(0,3) = msg.pose.position.x;
    homogeneous_tf(1,3) = msg.pose.position.y;
    homogeneous_tf(2,3) = msg.pose.position.z;

    return homogeneous_tf;
}

/**
 * returns the homogeneous transformation representing the offset between the offset and the apriltag
*/
Eigen::Matrix4d get_offset_to_apriltag(const Bucket bucket) {
    Eigen::Matrix4d H_offset_apriltag;

    switch (bucket) {
        case Bucket::Bucket1:
            H_offset_apriltag << -0.999663, 0.031221, 0.000515, 0.550040, 0.031171, 0.998714, -0.039981, -0.516124,  -0.001763, -0.039951, -0.999351, -0.407456, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket1a:
            H_offset_apriltag << -0.626168, -0.779510, 0.024069, 0.504049,  -0.779751, 0.625586, -0.025125, 0.206124,  0.004528, -0.034500, -0.999546, -0.422530,  0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket2:
            H_offset_apriltag << -0.999803, -0.003898, -0.026100, 0.490166, -0.003227, 0.999664, -0.025707, 0.407937, 0.026191, -0.025618, -0.999480, -0.399073, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket2a:
            H_offset_apriltag << -0.493306, 0.868041, -0.058797, 0.480437,  0.869885, 0.492477, -0.027707, -0.300816, 0.004905, -0.064814, -0.998037, -0.394514, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket3:
            H_offset_apriltag << 0.998840, 0.014387, -0.049128, 0.529951, 0.014442, -0.999895, 0.000803, -0.433028, -0.049111, -0.001512, -0.998943, -0.421869, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket3a:
            H_offset_apriltag << 0.705964, 0.706634, -0.050850, 0.414134, 0.702881, -0.707502, -0.073479, 0.207773, -0.087899, 0.016132, -0.996150, -0.413958, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket4:
            H_offset_apriltag << 0.993789, 0.002955, -0.112592, 0.430355, 0.000168, -0.999694, -0.024749, 0.422722, -0.112631, 0.024576, -0.993485, -0.411195, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
        case Bucket::Bucket4a:
            H_offset_apriltag << 0.585685, -0.807392, -0.073439, 0.485660, -0.806571, -0.589312, 0.046417, -0.207583, -0.080755, 0.032048, -0.996370, -0.404097, 0.000000, 0.000000, 0.000000, 1.000000;
            break;
    }
    
    return H_offset_apriltag;
}

/**
 * returns true if the drone's position and yaw are equal (within some tolerance) to the desired position and yaw
*/
bool drone_is_approximately_at_offset(const Eigen::Matrix4d& H_inertial_body, const Eigen::Matrix4d& H_inertial_offset, const double yaw_tolerance, const double position_component_tolerance) {
    // compute the yaw of the drone in the inertial frame
    Eigen::Vector3d euler_inertial_body = H_inertial_body.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
    double yaw_inertial_body = euler_inertial_body[0];

    // compute the yaw of the offset in the inertial frame
    Eigen::Vector3d euler_inertial_offset = H_inertial_offset.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
    double yaw_inertial_offset = euler_inertial_offset[0];

    // get the position of the drone in the inertial frame
    double x_inertial_body = H_inertial_body(0,3);
    double y_inertial_body = H_inertial_body(1,3);
    double z_inertial_body = H_inertial_body(2,3);

    // get the position of the offset in the inertial frame
    double x_inertial_offset = H_inertial_offset(0,3);
    double y_inertial_offset = H_inertial_offset(1,3);
    double z_inertial_offset = H_inertial_offset(2,3);

    // if the absolute value between the yaw and positions of the drone in the inertial frame and the offset in the inertial 
    // frame are all less than some tolerance, then that means the drone is approximately at the reference point, so return true
    bool x_within_tolerance = std::abs(x_inertial_body - x_inertial_offset) < position_component_tolerance;
    bool y_within_tolerance = std::abs(y_inertial_body - y_inertial_offset) < position_component_tolerance;
    bool z_within_tolerance = std::abs(z_inertial_body - z_inertial_offset) < position_component_tolerance;
    bool yaw_within_tolerance = std::abs(yaw_inertial_body - yaw_inertial_offset) < yaw_tolerance;

    // only return true if all of the abolute values are within the tolerances
    return x_within_tolerance && y_within_tolerance && z_within_tolerance && yaw_within_tolerance;
}

std::string bucket_to_string(const Bucket bucket) {
    std::string ret = "";

    switch(bucket) {
        case Bucket::Bucket1:
            return ret + "Bucket 1";
        case Bucket::Bucket1a:
            return ret + "Bucket 1a";
        case Bucket::Bucket2:
            return ret + "Bucket 2";
        case Bucket::Bucket2a:
            return ret + "Bucket 2a";
        case Bucket::Bucket3:
            return ret + "Bucket 3";
        case Bucket::Bucket3a:
            return ret + "Bucket 3a";
        case Bucket::Bucket4:
            return ret + "Bucket 4";
        case Bucket::Bucket4a:
            return ret + "Bucket 4a";
    }

}