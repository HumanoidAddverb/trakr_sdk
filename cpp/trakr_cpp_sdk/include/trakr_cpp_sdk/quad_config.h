/**
 * @file quad_config.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief CONFIG_SET containing configs for the entire
 * robot. (Refer to README.md)
 * @version 1.0
 * @date 2025-10-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef QUAD_CONFIG_H_
#define QUAD_CONFIG_H_

#include "motion_configs.h"
#include "safety_config.h"

namespace QuadDataTypes
{
    struct ROBOT_CONFIG
    {
        bool shutdown = false;
        bool exit = false;
        bool killed = false;

        void doExit()
        {
            shutdown = true;
            exit = true;
        }

        void doShutDown()
        {
            shutdown = true;
        }

        void kill()
        {
            killed = true;
        }
    };

    typedef struct
    {
        ROBOT_CONFIG master;
        MotionDataTypes::CONFIG_SET motion;
        SafetyDataTypes::CONFIG_SET safety;
        
    } CONFIG_SET;
}

#endif