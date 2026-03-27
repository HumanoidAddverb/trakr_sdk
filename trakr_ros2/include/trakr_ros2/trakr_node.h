#ifndef TRAKR_ROS2_NODE_H_
#define TRAKR_ROS2_NODE_H_

#include <memory>
#include <vector>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "robot.h"

class TrakrROS2Node : public rclcpp::Node
{
public:
    TrakrROS2Node() : Node("trakr_ros2") 
    {
        this->declare_parameter("robot_ip", "127.0.0.1");
        this->declare_parameter("robot_port", 15251);

        setup_();
    };

    ~TrakrROS2Node(){};

private:
    /// @brief bool indicating if we have connected to robot
    bool is_connected_ = false;

    /// @brief pointer to the low-level sdk/robot/socket
    std::unique_ptr<Robot> trakr_;

    /// @brief timer to run the low-level sdk at steady frequency
    rclcpp::TimerBase::SharedPtr timer_;

    /// @brief publisher for joint state
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_;

    /// @brief publisher for imu data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;

    /// @brief publisher for body twist data in robot-aligned world frame (world frame moving along robot)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_twist_;

    /// @brief subscriber to listen velocity plan from user
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr plan_sub_;

    /// @brief services for listening for setting various configurations
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> srv_;

    /// @brief callback group isolating services from rest
    rclcpp::CallbackGroup::SharedPtr cb_group_service_;

    /// @brief callback group isolating wall timer callbacks from rest
    rclcpp::CallbackGroup::SharedPtr cb_group_timer_;

    /// @brief data types to store local copies to-and-fro robot
    QuadDataTypes::CONFIG_SET config_;
    AlliedDataTypes::Plan plan_;
    AlliedDataTypes::State state_;

    /// @brief to hold status of configuration received from robot
    int config_status_ = 0;

    /// @brief bool to hold if a new configuration has to be set in robot
    bool set_config_ = false;

    /// @brief mutex only over the configuration
    std::mutex mut_config_;

    /// @brief mutex only over the user plan
    std::mutex mut_plan_;

    void setup_();

    void setupRobot_();

    void createCallbackGroups_();

    void createDataStreams_();

    void createServices_();

    void run_();

    void planCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);

    void sendState_();

    void waitForResponse_(std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setAIMode1_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setAIMode2_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setClassicalMode_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setGestureMode_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setGestureStandUp_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
                    
    void setGestureSitDown_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setGestureLeftHandShake_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setGestureRightHandShake_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif