#include "trakr_node.h"

/**
 * @brief Setup the node
 *
 */
void TrakrROS2Node::setup_()
{
    // bringing the robot to life
    setupRobot_();

    // callback groups for services and wall timer
    createCallbackGroups_();

    // creating pub-sub for continuous data streams
    createDataStreams_();

    // creating services for configurations
    createServices_();

    // creating wall timer running the robot
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000), 
        std::bind(&TrakrROS2Node::run_, this),
        cb_group_timer_
    );

    if(!is_connected_)
    {
        RCLCPP_INFO(this->get_logger(), "Node failed to setup trakr");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Node setup completed");
}

/**
 * @brief Setup the robot (sdk)
 *
 */
void TrakrROS2Node::setupRobot_()
{
    // getting parameters from ros2 network
    std::string robot_ip = this->get_parameter("robot_ip").as_string();
    int robot_port = this->get_parameter("robot_port").as_int();
    RCLCPP_INFO(this->get_logger(), "Connecting to Trakr at %s:%d", robot_ip.c_str(), robot_port);

    // creating object for our robot
    trakr_ = std::make_unique<Robot>(robot_ip.c_str(), robot_port);

    // setting initial data to zero
    plan_.joint.kp = JointVector::Zero();
    plan_.joint.kd = JointVector::Zero();
    plan_.joint.pos = JointVector::Zero();
    plan_.joint.vel = JointVector::Zero();
    plan_.joint.tor = JointVector::Zero();

    plan_.torso.pos = Vector6::Zero();
    plan_.torso.vel = Vector6::Zero();

    // setting up the robot to give it life
    if(!trakr_->setup(config_, plan_))
    {
        is_connected_ = false;
    }
    is_connected_ = true;

    // just to get an updated (& valid) config from robot
    trakr_->getConfig(config_);
}

/**
 * @brief Creating the callback groups for node
 *
 */
void TrakrROS2Node::createCallbackGroups_()
{
    // callback groups for services and wall timer
    cb_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Creating the publishers/subscriptions for data stream
 *
 */
void TrakrROS2Node::createDataStreams_()
{
    // creating pub-sub for continuous data streams
    joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/trakr/joint_states", 10);
    imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/trakr/imu", 10);
    body_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/trakr/body_twist", 10);
    plan_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/trakr/cmd_vel", 10, std::bind(&TrakrROS2Node::planCallback_, this, std::placeholders::_1)
    );
}

/**
 * @brief Creating services for different configurations
 *
 */
void TrakrROS2Node::createServices_()
{
    // creating services for configurations
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/ai_mode/walk", 
            std::bind(&TrakrROS2Node::setAIMode1_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/ai_mode/climb", 
            std::bind(&TrakrROS2Node::setAIMode2_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/classical_mode/trot", 
            std::bind(&TrakrROS2Node::setClassicalMode_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/gesture_mode/orientation", 
            std::bind(&TrakrROS2Node::setGestureMode_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/gesture_mode/standup", 
            std::bind(&TrakrROS2Node::setGestureStandUp_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/gesture_mode/sitdown", 
            std::bind(&TrakrROS2Node::setGestureSitDown_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/gesture_mode/left_handshake", 
            std::bind(&TrakrROS2Node::setGestureLeftHandShake_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
    srv_.push_back(
        this->create_service<std_srvs::srv::Trigger>(
            "/trakr/gesture_mode/right_handshake", 
            std::bind(&TrakrROS2Node::setGestureRightHandShake_, this, std::placeholders::_1, std::placeholders::_2), 
            rclcpp::ServicesQoS(), 
            cb_group_service_
        )
    );
}

/**
 * @brief Running the core robot system loop (over sdk)
 *
 */
void TrakrROS2Node::run_()
{
    if((!is_connected_) || (!trakr_->run()))
    {
        RCLCPP_INFO(this->get_logger(), "Failed to run; shutting down network");
        rclcpp::shutdown();
        is_connected_ = false;
    }

    trakr_->getData(state_);
    mut_config_.lock();
    if(!set_config_)
    {
        trakr_->getConfig(config_);
    }
    mut_config_.unlock();

    sendState_();

    config_status_ = trakr_->getConfigStatus();

    mut_config_.lock();
    if(set_config_)
    {
        trakr_->setConfig(config_);
        set_config_ = false;
    }
    mut_config_.unlock();

    mut_plan_.lock();
    trakr_->setData(plan_);
    mut_plan_.unlock();
}

/**
 * @brief Publishing the states of robot over different topics
 *
 */
void TrakrROS2Node::sendState_()
{
    auto joint = sensor_msgs::msg::JointState();
    auto imu = sensor_msgs::msg::Imu();
    auto body_twist = geometry_msgs::msg::Twist();

    // creating JointState message
    joint.name = {
        "FL_adduction", "FL_hip", "FL_thigh",
        "FR_adduction", "FR_hip", "FR_thigh",
        "RL_adduction", "RL_hip", "RL_thigh",
        "RR_adduction", "RR_hip", "RR_thigh",
    };

    joint.header.stamp.sec = static_cast<int32_t>(state_.joint.timestamp / (unsigned long long)(1e9));
    joint.header.stamp.nanosec = static_cast<int32_t>(state_.joint.timestamp % (unsigned long long)(1e9));
    joint.position = std::vector<double>(state_.joint.pos.data(), state_.joint.pos.data() + state_.joint.pos.size());
    joint.velocity = std::vector<double>(state_.joint.vel.data(), state_.joint.vel.data() + state_.joint.vel.size());
    joint.effort = std::vector<double>(state_.joint.tor.data(), state_.joint.tor.data() + state_.joint.tor.size());

    joint_->publish(joint);

    // Creating Imu message
    imu.header.stamp.sec = static_cast<int32_t>(state_.imu.timestamp / (unsigned long long)(1e9));
    imu.header.stamp.nanosec = static_cast<int32_t>(state_.imu.timestamp % (unsigned long long)(1e9));
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(state_.imu.euler[2], Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(state_.imu.euler[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(state_.imu.euler[0], Eigen::Vector3f::UnitX());
    imu.orientation.w = q.w();
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();

    imu.angular_velocity.x = state_.imu.gyro[0];
    imu.angular_velocity.y = state_.imu.gyro[1];
    imu.angular_velocity.z = state_.imu.gyro[2];

    imu.linear_acceleration.x = state_.imu.acc[0];
    imu.linear_acceleration.y = state_.imu.acc[1];
    imu.linear_acceleration.z = state_.imu.acc[2];

    imu_->publish(imu);

    // Creating Twist message
    body_twist.angular.x = state_.torso.vel[0];
    body_twist.angular.y = state_.torso.vel[1];
    body_twist.angular.z = state_.torso.vel[2];
    body_twist.linear.x = state_.torso.vel[3];
    body_twist.linear.y = state_.torso.vel[4];
    body_twist.linear.z = state_.torso.vel[5];

    body_twist_->publish(body_twist);
}

/**
 * @brief Getting user plan through subscription callback
 *
 */
void TrakrROS2Node::planCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    mut_plan_.lock();
    plan_.torso.vel(2) = msg->angular.z;
    plan_.torso.vel(3) = msg->linear.x;
    plan_.torso.vel(4) = msg->linear.y;
    mut_plan_.unlock();
}

/**
 * @brief Wait for the robot to acknowledge the configuration
 * TODO: timeout waiting for acknowledgement for too long
 *
 */
void TrakrROS2Node::waitForResponse_(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    while(!(config_status_ > 1) && rclcpp::ok())
    {
        rclcpp::sleep_for(std::chrono::microseconds(1000));
    }

    if(config_status_ == 2)
    {
        response->success = true;
        rclcpp::sleep_for(std::chrono::milliseconds(1250));
    }
    else if(config_status_ == 3)
    {
        response->success = false;
    }
}

/**
 * @brief Setting configuration corresponding to AIModeWalk
 *
 */
void TrakrROS2Node::setAIMode1_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eMotion;
    config_.motion.strategy.type = MotionModes::eAIMode;
    config_.motion.strategy.seq = MotionGaits::eWalk;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to AIModeClimb
 *
 */
void TrakrROS2Node::setAIMode2_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eMotion;
    config_.motion.strategy.type = MotionModes::eAIMode;
    config_.motion.strategy.seq = MotionGaits::eClimb;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to ClassicalModeTrot
 *
 */
void TrakrROS2Node::setClassicalMode_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eMotion;
    config_.motion.strategy.type = MotionModes::eClassicalMode;
    config_.motion.strategy.seq = MotionGaits::eTrot;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to GestureMode-Reorientation
 *
 */
void TrakrROS2Node::setGestureMode_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eGesture;
    config_.motion.sequence.type = GestureTypes::eOrientation;
    config_.motion.sequence.seq = 0;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to GestureMode-StandUp
 *
 */
void TrakrROS2Node::setGestureStandUp_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eGesture;
    config_.motion.sequence.type = GestureTypes::eStandUp;
    config_.motion.sequence.seq = 0;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to GestureMode-SitDown
 *
 */
void TrakrROS2Node::setGestureSitDown_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eGesture;
    config_.motion.sequence.type = GestureTypes::eSitDown;
    config_.motion.sequence.seq = 0;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to GestureMode-LeftHandShake
 *
 */
void TrakrROS2Node::setGestureLeftHandShake_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eGesture;
    config_.motion.sequence.type = GestureTypes::eLeftShakeHand;
    config_.motion.sequence.seq = 0;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief Setting configuration corresponding to GestureMode-RightHandShake
 *
 */
void TrakrROS2Node::setGestureRightHandShake_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    mut_config_.lock();
    config_.motion.planner = MotionDataTypes::TaskTypes::eGesture;
    config_.motion.sequence.type = GestureTypes::eRightShakeHand;
    config_.motion.sequence.seq = 0;
    set_config_ = true;
    mut_config_.unlock();

    waitForResponse_(response);
}

/**
 * @brief main code
 *
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<TrakrROS2Node> node_ = std::make_shared<TrakrROS2Node>();
    executor.add_node(node_);
    executor.spin();
    
    rclcpp::shutdown();

    return 0;
}