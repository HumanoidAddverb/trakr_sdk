/**
 * @file motion_configs.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Motion related robot configurations
 * Refer to README.md
 * @version 1.0
 * @date 2025-10-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef MOTION_CONFIGS_H_
#define MOTION_CONFIGS_H_

#include "motion_types.h"
#include "sequence_types.h"

namespace MotionDataTypes
{
    enum class TaskTypes
    {
        eGesture,
        eMotion,
    };

    /// @brief basic config for all MotionPlannerInterface
    struct BaseConfig
    {
        int type;
        int seq;
    };

    struct MotionConfig : BaseConfig
    {
        MotionModes type = MotionModes::eClassicalMode;
        MotionGaits seq = MotionGaits::eStance;
    };

    struct GestureConfig : BaseConfig
    {
        GestureTypes type = GestureTypes::eStandUp;
        int seq = 0;
    };

    struct CONFIG_SET
    {
        TaskTypes planner = TaskTypes::eGesture;

        GestureConfig sequence;
        MotionConfig strategy;

    };
    
};

#endif