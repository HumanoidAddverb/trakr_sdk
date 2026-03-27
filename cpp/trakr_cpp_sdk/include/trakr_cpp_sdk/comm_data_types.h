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
#include <stdint.h>

#define NDOF 12

/**
 * @brief Holds the continuous commands + config
 * for network communication
 */
namespace SocketDataTypes
{
    struct JointState
    {
        std::array<float,NDOF> pos {0};
        std::array<float,NDOF> vel {0};
        std::array<float,NDOF> tor {0};
        std::array<float,NDOF> kp {0};
        std::array<float,NDOF> kd {0};
        uint64_t timestamp = 0;
    };

    struct TorsoState
    {
        std::array<float,6> pos {0};
        std::array<float,6> vel {0};
        uint64_t timestamp = 0;
    };

    struct IMUState
    {
        std::array<float,3> acc {0};
        std::array<float,3> gyro {0};
        std::array<float,3> mag {0};
        std::array<float,3> euler {0};
        uint64_t timestamp = 0;
    };

    struct BatteryState
    {
        float voltage = 0;
        float current = 0;
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