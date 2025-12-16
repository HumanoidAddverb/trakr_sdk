/**
 * @file comm_data_types.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Packet Definitions - AlliedDataTypes
 * @version 1.0
 * @date 2025-10-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef COMM_DATA_TYPES_H_
#define COMM_DATA_TYPES_H_

#include <array>
#include "comm_config.h"

#define NDOF 12

/**
 * @brief Holds the continuous commands + config
 * for network communication
 */
namespace SocketDataTypes
{
    struct JointState
    {
        std::array<float,NDOF> pos;
        std::array<float,NDOF> vel;
        std::array<float,NDOF> tor;
        std::array<float,NDOF> kp;
        std::array<float,NDOF> kd;
    };

    struct TorsoState
    {
        std::array<float,6> pos;
        std::array<float,6> vel;
    };

    struct IMUState
    {
        std::array<float,3> acc;
        std::array<float,3> gyro;
        std::array<float,3> mag;
        std::array<float,3> euler;
    };

    struct BatteryState
    {
        float voltage;
        float current;
    };

    struct Plan
    {
        /// @brief High level commands
        TorsoState torso;
        
        /// @brief Low level commands
        JointState joint;
    };

    struct State
    {
        /// @brief EKF Torso Pose estimate
        TorsoState torso;

        /// @brief Joint data
        JointState joint;

        /// @brief imu data
        IMUState imu;

        /// @brief battery data
        BatteryState power;
    };

    struct AlliedPlan
    {
        Plan plan;
        Config config;
    };

    struct AlliedState
    {
        State state;
        Config config;
    };
};



#endif