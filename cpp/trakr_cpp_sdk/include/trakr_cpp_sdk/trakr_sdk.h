/**
 * @file trakr_sdk.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Client-end SDK for writing C++ applications
 * @version 1.0
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef TRAKR_SDK_H_
#define TRAKR_SDK_H_

#include "comm_data_types.h"
#include "network_interface.h"

namespace ClientNetworkConfig
{
    /// @brief Data type to be used as Tx
    typedef SocketDataTypes::AlliedState AlliedState;

    /// @brief Data type to be used as Rx
    typedef SocketDataTypes::AlliedPlan AlliedPlan;

    class RobotNetwork
    {
        public:

        RobotNetwork() {};

        ~RobotNetwork() {};

        /// @brief setup the network before connection
        /// @return
        bool setup();

        /// @brief connect with the peer
        /// @return
        bool connect(const char* ip_addr, const int port);

        /// @brief close the connection
        /// @return
        bool disconnect();

        /// @brief connection status
        /// @return
        bool isConnected();

        /// @brief shutdown connection
        /// @return
        bool shutdown();

        /// @brief read from network
        /// @return
        bool readData();

        /// @brief get data
        /// @param
        /// @return
        bool getData(AlliedState &recv_data);

        /// @brief set data to be sent
        /// @param
        /// @return
        bool setData(const AlliedPlan &send_data);

        /// @brief send to network
        /// @return
        bool writeData();

        private:

        /// @brief Pointer to our network implementation
        NetworkInterface<AlliedPlan, AlliedState>* net_;
    };

}

#endif
