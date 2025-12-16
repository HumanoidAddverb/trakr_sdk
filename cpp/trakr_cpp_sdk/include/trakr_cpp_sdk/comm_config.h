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
        bool shutdown;
        bool exit;
        bool killed;
    };


    // MotionDataTypes config
    struct MotionBaseConfig
    {
        int type;
        int seq;
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
        bool enabled;
    };
    

    struct LLSafetyConfig : SafetyBaseConfig
    {
        bool jpos;
        bool jvel;
        bool jtor;
        bool dtor;

        std::array<float, NDOF> lim_jpos_min;
        std::array<float, NDOF> lim_jpos_max;
        std::array<float, NDOF> lim_jvel;
        std::array<float, NDOF> lim_jtor;
        std::array<float, NDOF> lim_dtor;
    };

    struct LLHeartbeatConfig : SafetyBaseConfig
    {
        int init_disable_time;

        bool enable_CAN;
        int CAN_timeout;

        bool observe_jpos;
        bool observe_jvel;
        bool observe_jtor;

        bool enable_IMU;
        int IMU_timeout;
        bool observe_accel;
        bool observe_gyro;
    };

    struct TorsoSafetyConfig : SafetyBaseConfig
    {
        bool observe_roll;
        float roll_limit;

        bool observe_pitch;
        float pitch_limit;

        bool observe_gyro;
        float gyro_limit;
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
        int status_ = -1; 

        RobotConfig master;
        MotionConfig motion;
        SafetyConfig safety;
    };
};



#endif