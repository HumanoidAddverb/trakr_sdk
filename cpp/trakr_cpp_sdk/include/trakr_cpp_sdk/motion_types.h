/**
 * @file motion_types.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Motion Types - Different Modes for Robot
 * Refer to README.md
 * @version 1.0
 * @date 2025-10-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef MOTION_DATA_TYPES_H_
#define MOTION_DATA_TYPES_H_

#include <variant>

/**
 * @brief Type of strategies for TO Motion Module
 * 
 */
enum class MotionModes
{
    eNone, // Do not use
    eClassicalMode, 
    eAIMode,
    eDeveloperMode,
}; 

/**
 * \@brief Type of available gaits (only in eClassicalMode)
 * 
 */
enum class MotionGaits 
{
    eStance, 
    eTrot, 
};

#endif