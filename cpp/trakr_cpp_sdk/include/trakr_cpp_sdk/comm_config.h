/**
 * @file comm_config.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Packet Definitions - QuadConfig
 * @version 1.0
 * @date 2025-10-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef COMM_CONFIG_H_
#define COMM_CONFIG_H_

#define NDOF 12

#include <array>

/**
 * @brief Holds the continuous commands + config
 * for network communication
 */
namespace SocketDataTypes
{
    // Robot Master Config
    struct RobotConfig
    {
        bool shutdown = false;
        bool exit = false;
        bool killed = false;
    };


    // MotionDataTypes config
    struct MotionBaseConfig
    {
        int type = 0;
        int seq = 0;
    };

    struct MotionConfig
    {
        int planner;

        MotionBaseConfig sequence;
        MotionBaseConfig strategy;
    };

    // SafetyDataTypes config
    struct SafetyBaseConfig
    {
        bool enabled = false;
    };
    

    struct LLSafetyConfig : SafetyBaseConfig
    {
        bool jpos = false;
        bool jvel = false;
        bool jtor = false;
        bool dtor = false;

        std::array<float, NDOF> lim_jpos_min {0};
        std::array<float, NDOF> lim_jpos_max {0};
        std::array<float, NDOF> lim_jvel {0};
        std::array<float, NDOF> lim_jtor {0};
        std::array<float, NDOF> lim_dtor {0};
    };

    struct LLHeartbeatConfig : SafetyBaseConfig
    {
        int init_disable_time = 0;

        bool enable_CAN = false;
        int CAN_timeout = 0;

        bool observe_jpos = false;
        bool observe_jvel = false;
        bool observe_jtor = false;

        bool enable_IMU = false;
        int IMU_timeout = 0;
        bool observe_accel = false;
        bool observe_gyro = false;
    };

    struct TorsoSafetyConfig : SafetyBaseConfig
    {
        bool observe_roll = false;
        float roll_limit = 0;

        bool observe_pitch = false;
        float pitch_limit = 0;

        bool observe_gyro = false;
        float gyro_limit = 0;
    };
    
    struct SafetyConfig
    {
        LLSafetyConfig llsafety;
        LLHeartbeatConfig llheartbeat;
        TorsoSafetyConfig torso;
    };


    // CONFIG_SET
    struct Config
    {
        // Register used for config setting and acknowledgement
        int status = -1; 

        RobotConfig master;
        MotionConfig motion;
        SafetyConfig safety;
    };
};



#endif