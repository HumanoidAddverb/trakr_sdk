/**
 * @file network_interface.h
 * @author Addverb Technologies (humanoid@addverb.com)
 * @brief Headers for socket SDK of trakr
 * @version 1.0
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef NETWORK_INTERFACE_H_
#define NETWORK_INTERFACE_H_

template <typename Tx, typename Rx>
class NetworkInterface
{
    public:

    ~NetworkInterface() {};

    /// @brief setup the network before connection
    /// @return
    virtual bool setup() = 0;

    /// @brief connect with the peer
    /// @return
    virtual bool connect(const char* ip_addr, const int port) = 0;

    /// @brief close the connection
    /// @return
    virtual bool disconnect() = 0;

    /// @brief connection status
    /// @return
    virtual bool isConnected() = 0;

    /// @brief shutdown connection
    /// @return
    virtual bool shutdown() = 0;

    /// @brief read from network
    /// @return
    virtual bool readData() = 0;

    /// @brief get data
    /// @param
    /// @return
    virtual bool getData(Rx &recv_data) = 0;

    /// @brief set data to be sent
    /// @param
    /// @return
    virtual bool setData(const Tx &send_data) = 0;

    /// @brief send to network
    /// @return
    virtual bool writeData() = 0;
};

#endif