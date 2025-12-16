/**
 * @file robot.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Main Class to act as our Robot
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#ifndef ROBOT_H_
#define ROBOT_H_

#include <unistd.h>
#include <mutex>

#include "trakr_sdk.h"
#include "quad_config.h"
#include "allied_data_types.h"

typedef Eigen::Matrix<double, NDOF, 1> JointVector;
typedef Eigen::Matrix<double, 6, 1> Vector6;

/**
 * @brief Our Robot Wrapper
 * 
 */
class Robot 
{
    public :

    Robot(const char* ip_addr, const int port) 
    {
        ip_addr_ = ip_addr;
        port_ = port;
    };

    /// @brief Shutdown network before destructing
    ~Robot()
    {
        if(net_.isConnected())
        {
            net_.shutdown();
        }
    }

    bool setup(QuadDataTypes::CONFIG_SET &config, AlliedDataTypes::Plan &plan);

    bool run();

    /// @brief return network status
    /// @return 
    bool isAlive()
    {
        return net_.isConnected();
    }

    bool setData(AlliedDataTypes::Plan &plan);

    bool getData(AlliedDataTypes::State &state);

    bool setConfig(QuadDataTypes::CONFIG_SET &config);

    bool getConfig(QuadDataTypes::CONFIG_SET &config);

    int getConfigStatus();

    private :

    const char* ip_addr_;

    int port_;

    /// @brief Main robot network for communication
    ClientNetworkConfig::RobotNetwork net_;

    /// @brief Local buffer to send data
    ClientNetworkConfig::AlliedPlan plan_;

    /// @brief Local buffer to receive data
    ClientNetworkConfig::AlliedState state_;

    /// @brief Bool to decide if we have a new config and has to be set
    bool set_config_;

    /// @brief Response for last "setConfig" action
    int config_status_;

    void dataAdaptor_(ClientNetworkConfig::AlliedPlan &send, AlliedDataTypes::Plan &plan);

    void dataAdaptor_(ClientNetworkConfig::AlliedPlan &send, QuadDataTypes::CONFIG_SET &config);

    void dataAdaptor_(ClientNetworkConfig::AlliedState &recv, AlliedDataTypes::State &state);

    void dataAdaptor_(ClientNetworkConfig::AlliedState &recv, QuadDataTypes::CONFIG_SET &config);

    bool bringUp_();
};







#endif 
