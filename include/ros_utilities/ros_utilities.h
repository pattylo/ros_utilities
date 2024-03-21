#ifndef ROS_UTILITIES_H
#define ROS_UTILITIES_H

#include "essential.h"
#include <type_traits>


class RosUtilities 
{
public:
    RosUtilities();

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

    template <typename T_input, typename T_result>
    void ComputeMean(const T_input& data, T_result& return_value)
    // calculate the mean of a vector of vector
    {
        assert(!data.empty());

        return_value = std::accumulate(
            data.begin(), 
            data.end(), 
            T_result::Zero().eval(),
            [](const T_result& accum, const typename T_input::value_type& element) 
            {
                return accum + element;
            }
        ) / data.size();
    };

    template <typename T_input, typename T_result>
    void ComputeVariance(const T_input& data, T_result& return_value)
    {
        assert(!data.empty()); 

        // Calculate the mean
        T_result mean;
        ComputeMean(data, mean);

        return_value.setZero();

        for (const auto& element : data) {
            T_result diff = element - mean;
            return_value += diff.cwiseAbs2().eval(); // Element-wise square
        }

        return_value /= (data.size() + 1);
    }

    template <typename T_save_data>
    void write_yaml()
    
};

#endif // ROS_UTILITIES_H