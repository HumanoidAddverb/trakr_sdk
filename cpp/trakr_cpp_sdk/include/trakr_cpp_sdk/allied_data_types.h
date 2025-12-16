/**
 * @file allied_data_types.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Allied Data Types; data streams between robot and client
 * Refer to README.md
 * @version 1.0
 * @date 2025-10-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef ALLIED_DATA_TYPES_H_
#define ALLIED_DATA_TYPES_H_

#include <eigen3/Eigen/Core>

/**
 * @brief Holds the continuous commands
 * (cartesian and joint-space)
 */
namespace AlliedDataTypes
{
    struct JointState
    {
        Eigen::Matrix<double, 12, 1> pos = Eigen::Matrix<double, 12, 1>::Zero();
        Eigen::Matrix<double, 12, 1> vel = Eigen::Matrix<double, 12, 1>::Zero();
        Eigen::Matrix<double, 12, 1> tor = Eigen::Matrix<double, 12, 1>::Zero();
        Eigen::Matrix<double, 12, 1> kp = Eigen::Matrix<double, 12, 1>::Zero();
        Eigen::Matrix<double, 12, 1> kd = Eigen::Matrix<double, 12, 1>::Zero();
    };

    struct TorsoState
    {
        Eigen::Matrix<double, 6, 1> pos = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> vel = Eigen::Matrix<double, 6, 1>::Zero();
    };

    struct IMUState
    {
        Eigen::Matrix<double, 3, 1> acc = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> gyro = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> mag = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 1> euler = Eigen::Matrix<double, 3, 1>::Zero();
    };

    struct BatteryState
    {
        double voltage;
        double current;
    };

    struct Plan
    {
        /// @brief High Level Commands
        TorsoState torso;

        /// @brief Low Level Commands
        JointState joint;
    };
    
    struct State
    {
        /// @brief EKF-based torso pose estimation
        TorsoState torso;

        /// @brief Joint-state sensor feedback
        JointState joint;

        /// @brief IMU Sensor feedback
        IMUState imu;

        /// @brief Power feedback from battery
        BatteryState power;
    };
};

#endif