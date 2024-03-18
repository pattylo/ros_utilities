#ifndef ROS_UTILITIES_H
#define ROS_UTILITIES_H

#include "essential.h"

class RosUtilities 
{
public:
    RosUtilities();  // Constructor
    void myFunction();  // Example member function

    Eigen::Vector3d q2rpy(const Eigen::Quaterniond& q);
    Eigen::Quaterniond rpy2q(const Eigen::Vector3d& rpy);
    Eigen::Vector3d q_rotate_vector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
    
    nav_msgs::Odometry SE3_to_odommsg(
        const Sophus::SE3d& pose_on_SE3,
        const Sophus::Vector6d& velo,
        const std_msgs::Header& msgHeader
    );
    geometry_msgs::PoseStamped SE3_to_posemsg(
        const Sophus::SE3d& pose_on_SE3, 
        const std_msgs::Header& msgHeader
    );

    Sophus::SE3d posemsg_to_SE3(const geometry_msgs::Pose& pose);
    Sophus::Vector6d twistmsg_to_velo(const geometry_msgs::Twist& twist);
    Sophus::Vector6d imumsg_to_accl(const sensor_msgs::Imu& imu);

    Sophus::Matrix3d Jacobi3dR(const Sophus::SE3d& pose);
};

#endif // ROS_UTILITIES_H