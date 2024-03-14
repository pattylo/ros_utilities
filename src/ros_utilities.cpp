#include "ros_utilities/ros_utilities.h"

// my_class.cpp

RosUtilities::RosUtilities() {
    // Constructor implementation
}

void RosUtilities::myFunction() {
    // Function implementation
}

Eigen::Vector3d RosUtilities::q2rpy(const Eigen::Quaterniond& q)
{
    tfScalar yaw, pitch, roll;
    tf::Quaternion q_tf;
    q_tf.setW(q.w());
    q_tf.setX(q.x());
    q_tf.setY(q.y());
    q_tf.setZ(q.z());

    tf::Matrix3x3 mat(q_tf);
    mat.getEulerYPR(yaw, pitch, roll);

    return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond RosUtilities::rpy2q(const Eigen::Vector3d& rpy)
{
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}

Eigen::Vector3d RosUtilities::q_rotate_vector(
    const Eigen::Quaterniond& q, 
    const Eigen::Vector3d& v
)
{
    return q * v;
}

nav_msgs::Odometry RosUtilities::SE3_to_odommsg(
    const Sophus::SE3d& pose_on_SE3,
    const Sophus::Vector6d& velo,
    const std_msgs::Header& msgHeader
)
{
    nav_msgs::Odometry returnOdomMsg;
    returnOdomMsg.header = msgHeader;

    // pose
    returnOdomMsg.pose.pose.position.x = pose_on_SE3.translation().x();
    returnOdomMsg.pose.pose.position.y = pose_on_SE3.translation().y();
    returnOdomMsg.pose.pose.position.z = pose_on_SE3.translation().z();

    returnOdomMsg.pose.pose.orientation.w = pose_on_SE3.unit_quaternion().w();
    returnOdomMsg.pose.pose.orientation.x = pose_on_SE3.unit_quaternion().x();
    returnOdomMsg.pose.pose.orientation.y = pose_on_SE3.unit_quaternion().y();
    returnOdomMsg.pose.pose.orientation.z = pose_on_SE3.unit_quaternion().z();

    // twist
    returnOdomMsg.twist.twist.linear.x = velo(0);
    returnOdomMsg.twist.twist.linear.y = velo(1);
    returnOdomMsg.twist.twist.linear.z = velo(2);

    returnOdomMsg.twist.twist.angular.x = velo(3);
    returnOdomMsg.twist.twist.angular.y = velo(4);
    returnOdomMsg.twist.twist.angular.z = velo(5);

    return returnOdomMsg;
}

geometry_msgs::PoseStamped RosUtilities::SE3_to_posemsg(
    const Sophus::SE3d& pose_on_SE3,
    const std_msgs::Header& msgHeader
)
{
    geometry_msgs::PoseStamped returnPoseMsg;
    returnPoseMsg.header = msgHeader;

    returnPoseMsg.pose.position.x = pose_on_SE3.translation().x();
    returnPoseMsg.pose.position.y = pose_on_SE3.translation().y();
    returnPoseMsg.pose.position.z = pose_on_SE3.translation().z();

    returnPoseMsg.pose.orientation.w = pose_on_SE3.unit_quaternion().w();
    returnPoseMsg.pose.orientation.x = pose_on_SE3.unit_quaternion().x();
    returnPoseMsg.pose.orientation.y = pose_on_SE3.unit_quaternion().y();
    returnPoseMsg.pose.orientation.z = pose_on_SE3.unit_quaternion().z();

    return returnPoseMsg;
}

Sophus::SE3d RosUtilities::posemsg_to_SE3(
    const geometry_msgs::Pose& pose
)
{
    return Sophus::SE3d(
        Eigen::Quaterniond(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z  
        ).normalized().toRotationMatrix(),
        Eigen::Translation3d(
            pose.position.x,
            pose.position.y,
            pose.position.z
        ).translation()
    );
}

Sophus::Vector6d RosUtilities::twistmsg_to_velo(
    const geometry_msgs::Twist& twist
)
{
    return (
        Sophus::Vector6d() << 
        twist.linear.x,
        twist.linear.y, 
        twist.linear.z, 
        twist.angular.x, 
        twist.angular.y, 
        twist.angular.z
    ).finished();
}
