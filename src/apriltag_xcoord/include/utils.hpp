#include <Eigen/Dense>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <cmath> // for std::pow

enum Bucket {
    Bucket1,
    Bucket1a,
    Bucket2,
    Bucket2a,
    Bucket3,
    Bucket3a,
    Bucket4,
    Bucket4a
};

std::string homogeneous_tf_to_string(const Eigen::Matrix4d&);
geometry_msgs::PoseStamped homogeneous_tf_to_geo_msg_pose_stamped(const Eigen::Matrix4d&);
Eigen::Matrix4d geo_msg_pose_stamped_to_homogeneous_tf(geometry_msgs::PoseStamped&);
Eigen::Matrix4d get_offset_to_apriltag(const Bucket);
bool drone_is_approximately_at_offset(const Eigen::Matrix4d&, const Eigen::Matrix4d&, const double, const double);

std::string bucket_to_string(const Bucket);