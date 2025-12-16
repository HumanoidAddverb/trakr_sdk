/**
 * @file safety_config.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Configurations for safeties in robot
 * Refer to README.md
 * @version 1.0
 * @date 2025-10-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef SAFETY_CONFIG_H_
#define SAFETY_CONFIG_H_

#include <eigen3/Eigen/Core>
#include "safety_constants.h"
#include <memory>

/**
 * @brief enum holding the all the SafetyInterface(s) available
 * 
 */
enum class SafetyTypes
{
    LLSafety,
    LLHeartbeat,
    TorsoSafety,
};

/**
 * @brief namespace holding config definitions for all SafetyInterfaces
 * 
 */
namespace SafetyDataTypes
{
    // Common configs for all safeties
    struct BaseConfig
    {
        bool enabled = true;

        void enable()
        {
            enabled = true;
        }

        void disable()
        {
            enabled = false;
        }
    };

    // configs for LL_safety
    struct LLSafetyConfig : BaseConfig
    {
        bool jpos = true;
        bool jvel = true;
        bool jtor = false;
        bool dtor = true;

        bool check_nan = true;

        Eigen::Matrix<double, 12, 1> lim_jpos_min;
        Eigen::Matrix<double, 12, 1> lim_jpos_max;
        Eigen::Matrix<double, 12, 1> lim_jvel;
        Eigen::Matrix<double, 12, 1> lim_jtor;
        Eigen::Matrix<double, 12, 1> lim_dtor;

        LLSafetyConfig ()
        {
            lim_jpos_min[0] = SAFETY_J0_THRESHOLD_MIN;
            lim_jpos_min[1] = SAFETY_J1_THRESHOLD_MIN;
            lim_jpos_min[2] = SAFETY_J2_THRESHOLD_MIN;
            lim_jpos_min[3] = SAFETY_J3_THRESHOLD_MIN;
            lim_jpos_min[4] = SAFETY_J4_THRESHOLD_MIN;
            lim_jpos_min[5] = SAFETY_J5_THRESHOLD_MIN;
            lim_jpos_min[6] = SAFETY_J6_THRESHOLD_MIN;
            lim_jpos_min[7] = SAFETY_J7_THRESHOLD_MIN;
            lim_jpos_min[8] = SAFETY_J8_THRESHOLD_MIN;
            lim_jpos_min[9] = SAFETY_J9_THRESHOLD_MIN;
            lim_jpos_min[10] = SAFETY_J10_THRESHOLD_MIN;
            lim_jpos_min[11] = SAFETY_J11_THRESHOLD_MIN;

            lim_jpos_max[0] = SAFETY_J0_THRESHOLD_MAX;
            lim_jpos_max[1] = SAFETY_J1_THRESHOLD_MAX;
            lim_jpos_max[2] = SAFETY_J2_THRESHOLD_MAX;
            lim_jpos_max[3] = SAFETY_J3_THRESHOLD_MAX;
            lim_jpos_max[4] = SAFETY_J4_THRESHOLD_MAX;
            lim_jpos_max[5] = SAFETY_J5_THRESHOLD_MAX;
            lim_jpos_max[6] = SAFETY_J6_THRESHOLD_MAX;
            lim_jpos_max[7] = SAFETY_J7_THRESHOLD_MAX;
            lim_jpos_max[8] = SAFETY_J8_THRESHOLD_MAX;
            lim_jpos_max[9] = SAFETY_J9_THRESHOLD_MAX;
            lim_jpos_max[10] = SAFETY_J10_THRESHOLD_MAX;
            lim_jpos_max[11] = SAFETY_J11_THRESHOLD_MAX;

            lim_jvel[0] = SAFETY_VEL_J0_THRESHOLD;
            lim_jvel[1] = SAFETY_VEL_J1_THRESHOLD;
            lim_jvel[2] = SAFETY_VEL_J2_THRESHOLD;
            lim_jvel[3] = SAFETY_VEL_J3_THRESHOLD;
            lim_jvel[4] = SAFETY_VEL_J4_THRESHOLD;
            lim_jvel[5] = SAFETY_VEL_J5_THRESHOLD;
            lim_jvel[6] = SAFETY_VEL_J6_THRESHOLD;
            lim_jvel[7] = SAFETY_VEL_J7_THRESHOLD;
            lim_jvel[8] = SAFETY_VEL_J8_THRESHOLD;
            lim_jvel[9] = SAFETY_VEL_J9_THRESHOLD;
            lim_jvel[10] = SAFETY_VEL_J10_THRESHOLD;
            lim_jvel[11] = SAFETY_VEL_J11_THRESHOLD;

            lim_jtor[0] = SAFETY_TOR_J0_THRESHOLD;
            lim_jtor[1] = SAFETY_TOR_J1_THRESHOLD;
            lim_jtor[2] = SAFETY_TOR_J2_THRESHOLD;
            lim_jtor[3] = SAFETY_TOR_J3_THRESHOLD;
            lim_jtor[4] = SAFETY_TOR_J4_THRESHOLD;
            lim_jtor[5] = SAFETY_TOR_J5_THRESHOLD;
            lim_jtor[6] = SAFETY_TOR_J6_THRESHOLD;
            lim_jtor[7] = SAFETY_TOR_J7_THRESHOLD;
            lim_jtor[8] = SAFETY_TOR_J8_THRESHOLD;
            lim_jtor[9] = SAFETY_TOR_J9_THRESHOLD;
            lim_jtor[10] = SAFETY_TOR_J10_THRESHOLD;
            lim_jtor[11] = SAFETY_TOR_J11_THRESHOLD;

            lim_dtor = lim_jtor;
        }
    };

    // configs for LL_heartbeat
    struct LLHeartbeatConfig : BaseConfig
    {
        int init_disable_time = 2.5; // time after init for which safety is not performed

        bool enable_CAN = true;
        int CAN_timeout = 100;

        bool observe_jpos = false;
        bool observe_jvel = true;
        bool observe_jtor = true;

        bool enable_IMU = true;
        int IMU_timeout = 200;

        bool observe_accel = true;
        bool observe_gyro = false;

    };

    // configs for TorsoSafety
    struct TorsoSafetyConfig : BaseConfig
    {
        bool observe_roll = true;
        double roll_limit = 1.2;

        bool observe_pitch = true;
        double pitch_limit = 1.2;

        bool observe_gyro = false;
        double gyro_limit = 5.0;
    };


    typedef struct
    {
        LLSafetyConfig llsafety;
        LLHeartbeatConfig llheartbeat;
        TorsoSafetyConfig torso;

        void disableAll()
        {
            llsafety.disable();
            llheartbeat.disable();
            torso.disable();
        }

    } CONFIG_SET;

}


#endif