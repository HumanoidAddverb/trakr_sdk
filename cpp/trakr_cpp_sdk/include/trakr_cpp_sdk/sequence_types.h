/**
 * @file sequence_types.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Types of Gestures/Sequences
 * @version 1.0
 * @date 2025-10-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef SEQ_DATA_TYPES_H_
#define SEQ_DATA_TYPES_H_

#include <eigen3/Eigen/Core>
#include <iostream>

/**
 * @brief Type of sequences
 * 
 */
enum class GestureTypes
{
    eNone = 0,
    eStandUp = 1,
    eSitDown = 2,
    eOrientation = 3,
    eLeftShakeHand = 4,
    eRightShakeHand = 5,
    eReserved = 6,
};

//// STANDUP COORDINATES (For Reference)

#define SEQ_STANDUP_A1 0
#define SEQ_STANDUP_A2 0.81    
#define SEQ_STANDUP_A3 -1.57
#define SEQ_STANDUP_B1 0
#define SEQ_STANDUP_B2 0.81    
#define SEQ_STANDUP_B3 -1.57
#define SEQ_STANDUP_C1 0
#define SEQ_STANDUP_C2 0.81
#define SEQ_STANDUP_C3 -1.57
#define SEQ_STANDUP_D1 0
#define SEQ_STANDUP_D2 0.81
#define SEQ_STANDUP_D3 -1.57


/// SITDOWN COORDINATES (For Reference)

#define SEQ_SITDOWN_A1 0
#define SEQ_SITDOWN_A2 1.106787
#define SEQ_SITDOWN_A3 -2.63
#define SEQ_SITDOWN_B1 0
#define SEQ_SITDOWN_B2 1.106787
#define SEQ_SITDOWN_B3 -2.63
#define SEQ_SITDOWN_C1 0
#define SEQ_SITDOWN_C2 1.106787
#define SEQ_SITDOWN_C3 -2.63
#define SEQ_SITDOWN_D1 0
#define SEQ_SITDOWN_D2 1.106787
#define SEQ_SITDOWN_D3 -2.63


#endif