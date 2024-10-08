/*
    This file is part of ros_utilities - basic functions for ROS usage

    ros_utilities is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ros_utilities is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ros_utilities.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file ros_utilities.h
 * \date 16/03/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief frequently used libraries - header
*/

#ifndef ROS_UTILITIES_H
#define ROS_UTILITIES_H

#include "essential.h"
#include "RosTopicConfigs.hpp"


class ros_utilities : public RosTopicConfigs
{
public:
    ros_utilities();

    Eigen::Vector3d q2rpy(const Eigen::Quaterniond& q);
    Eigen::Quaterniond rpy2q(const Eigen::Vector3d& rpy);
    Eigen::Vector3d q_rotate_vector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
    Eigen::Vector3d SO3_rotate_vector(const Sophus::SO3d& ROTm, const Eigen::Vector3d& v);
    
    nav_msgs::Odometry SE3_to_odommsg(
        const Sophus::SE3d& pose_on_SE3,
        const Sophus::Vector6d& velo,
        const std_msgs::Header& msgHeader
    );
    geometry_msgs::PoseStamped SE3_to_posemsg(
        const Sophus::SE3d& pose_on_SE3, 
        const std_msgs::Header& msgHeader
    );
    Eigen::Vector4d SE3_to_flat(
        const Sophus::SE3d& pose_on_SE3
    );

    Sophus::SE3d posemsg_to_SE3(const geometry_msgs::Pose& pose);
    Sophus::Vector6d twistmsg_to_velo(const geometry_msgs::Twist& twist);
    Sophus::Vector6d imumsg_to_accl(const sensor_msgs::Imu& imu);

    geometry_msgs::Quaternion q2qmsg(const Eigen::Quaterniond q);
    geometry_msgs::Quaternion rpy2qmsg(const Eigen::Vector3d rpy);
    Eigen::Quaterniond Qmsg_to_Q(const geometry_msgs::Quaternion& Qmsg);

    Sophus::Matrix3d Jacobi3dR(const Sophus::SE3d& pose);

    template <typename T_input, typename T_result>
    void ComputeMeanVector(const T_input& data, T_result& return_value)
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

    template <typename T_input>
    void ComputeMeanScalar(const T_input& data, double& return_value)
    {
        return_value = std::accumulate(
            data.begin(),
            data.end(),
            0.0
        ) / data.size();
    };

    template <typename T_input, typename T_result>
    void ComputeVariance(const T_input& data, T_result& return_value)
    {
        assert(!data.empty()); 

        // Calculate the mean
        T_result mean;
        ComputeMeanVector(data, mean);

        return_value.setZero();

        for (const auto& element : data) {
            T_result diff = element - mean;
            return_value += diff.cwiseAbs2().eval(); // Element-wise square
        }

        return_value /= (data.size() + 1);
    }

    template<typename T>
    void write_yaml(
        const T& input_data, 
        const std::string& yaml_filename_,
        const std::string& param_name_
    ) 
    {
        YAML::Node yaml_config = YAML::LoadFile(yaml_filename_);
        yaml_config[param_name_] = input_data;

        std::ofstream yaml_of(yaml_filename_);
        yaml_of << yaml_config;
        yaml_of.close();
    }

    void write_yaml(
        const Eigen::Vector3d& input_data_, 
        const std::string& yaml_filename_,
        const std::string& param_name_
    );
};

#endif // ROS_UTILITIES_H