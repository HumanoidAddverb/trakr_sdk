#include "robot.h"

/**
 * @brief Perform network setup and connection
 *
 */
bool Robot::setup(QuadDataTypes::CONFIG_SET &config, AlliedDataTypes::Plan &plan)
{
    if(!net_.setup())
    {
        return false;
    }

    dataAdaptor_(plan_, config);
    dataAdaptor_(plan_, plan);
    plan_.config.status_ = false;

    if(!net_.setData(plan_))
    {
        return false;
    }

    if(!net_.connect(ip_addr_, port_))
    {
        return false;
    }

    std::cout << "[ROBOT] Connected!" << std::endl;

    bringUp_();

    return true;
}

/**
 * @brief Communicate with robot (send and receive)
 *
 */
bool Robot::run()
{
    if(!net_.readData())
    {
        std::cout << "[ROBOT] Failed to read data" << std::endl;
        return false;
    }

    if(!net_.getData(state_))
    {
        return false;
    }

    // Reset once acknowledged by server
    if((state_.config.status_ > 0))
    {
        plan_.config.status_ = 0;
        config_status_ = state_.config.status_;
        state_.config.status_ = 0;
    }

    if(!net_.setData(plan_))
    {
        return false;
    }

    if(!net_.writeData())
    {
        std::cout << "[ROBOT] Failed to send data" << std::endl;
        return false;
    }

    return true;
}


/**
 * @brief Checking if robot bringUp is done
 *
 */
bool Robot::bringUp_()
{
    std::cout << "[ROBOT] Waiting for Robot to BringUp!" << std::endl;

    state_.config.status_ = -1;

    QuadDataTypes::CONFIG_SET config;

    int ctr = 0;

    while(state_.config.status_ < 0)
    {
        ctr++;

        if(!run())
        {
            std::cout << "[ROBOT] Failed in BringUp" << std::endl;
            return false;
        }

        if(ctr < 10)
        {
            usleep(100000);
            continue;
        }

        // Some internal state machine to check status
        if(state_.config.status_ == -2)
        {
            getConfig(config);
            setConfig(config);
            plan_.config.status_ = 1;
        }
        else
        {
            plan_.config.status_ = 0;
        }
        usleep(100000);
    }

    plan_.config.status_ = 0;

    std::cout << "[ROBOT] Robot BringUp Completed!" << std::endl;
    return true;
}

/**
 * @brief Set the allied data to be sent
 *
 */
bool Robot::setData(AlliedDataTypes::Plan &plan)
{
    dataAdaptor_(plan_, plan);
    return true;
}

/**
 * @brief Get the received allied data
 *
 */
bool Robot::getData(AlliedDataTypes::State &state)
{
    dataAdaptor_(state_, state);
    return true;
}

/**
 * @brief Set the robot config to be sent
 *
 */
bool Robot::setConfig(QuadDataTypes::CONFIG_SET &config)
{
    dataAdaptor_(plan_, config);
    plan_.config.status_ = 1;
    return true;
}

/**
 * @brief Get the latest config most probably being run by robot
 *
 */
bool Robot::getConfig(QuadDataTypes::CONFIG_SET &config)
{
    dataAdaptor_(state_, config);
    return true;
}

/**
 * @brief Perform network setup and connection
 *
 */
int Robot::getConfigStatus()
{
    int ret = config_status_;
    config_status_ = 0;
    return ret;
}

/**
 * @brief Set allied data into local buffer
 *
 */
void Robot::dataAdaptor_(ClientNetworkConfig::AlliedPlan &send, AlliedDataTypes::Plan &plan)
{
    for(int i = 0; i < NDOF; i++)
    {
        send.plan.joint.pos[i] = plan.joint.pos[i];
        send.plan.joint.vel[i] = plan.joint.vel[i];
        send.plan.joint.tor[i] = plan.joint.tor[i];
        send.plan.joint.kp[i] = plan.joint.kp[i];
        send.plan.joint.kd[i] = plan.joint.kd[i];
    }

    for(int i = 0; i < 6; i++)
    {
        send.plan.torso.pos[i] = plan.torso.pos[i];
        send.plan.torso.vel[i] = plan.torso.vel[i];
    }
}

/**
 * @brief Set config into local buffer
 *
 */
void Robot::dataAdaptor_(ClientNetworkConfig::AlliedPlan &send, QuadDataTypes::CONFIG_SET &config)
{
    send.config.master.shutdown = config.master.shutdown;
    send.config.master.exit = config.master.exit;
    send.config.master.killed = config.master.killed;

    send.config.motion.planner = static_cast<int>(config.motion.planner);
    send.config.motion.sequence.type = static_cast<int>(config.motion.sequence.type);
    send.config.motion.sequence.seq = static_cast<int>(config.motion.sequence.seq);
    send.config.motion.strategy.type = static_cast<int>(config.motion.strategy.type);
    switch (config.motion.strategy.type)
    {
        case MotionModes::eClassicalMode :
        send.config.motion.strategy.seq = static_cast<int>(config.motion.strategy.seq);
        break;

        default:
        send.config.motion.strategy.seq = 0;
        break;
    }

    send.config.safety.llsafety.enabled = config.safety.llsafety.enabled;
    send.config.safety.llsafety.jpos = config.safety.llsafety.jpos;
    send.config.safety.llsafety.jvel = config.safety.llsafety.jvel;
    send.config.safety.llsafety.jtor = config.safety.llsafety.jtor;
    send.config.safety.llsafety.dtor = config.safety.llsafety.dtor;

    for(int i = 0; i < NDOF; i++)
    {
        send.config.safety.llsafety.lim_jpos_min[i] = config.safety.llsafety.lim_jpos_min[i];
        send.config.safety.llsafety.lim_jpos_max[i] = config.safety.llsafety.lim_jpos_max[i];
        send.config.safety.llsafety.lim_jvel[i] = config.safety.llsafety.lim_jvel[i];
        send.config.safety.llsafety.lim_jtor[i] = config.safety.llsafety.lim_jtor[i];
        send.config.safety.llsafety.lim_dtor[i] = config.safety.llsafety.lim_dtor[i];
    }

    send.config.safety.llheartbeat.enabled = config.safety.llheartbeat.enabled;
    send.config.safety.llheartbeat.init_disable_time = config.safety.llheartbeat.init_disable_time;
    send.config.safety.llheartbeat.enable_CAN = config.safety.llheartbeat.enable_CAN;
    send.config.safety.llheartbeat.CAN_timeout = config.safety.llheartbeat.CAN_timeout;
    send.config.safety.llheartbeat.observe_jpos = config.safety.llheartbeat.observe_jpos;
    send.config.safety.llheartbeat.observe_jvel = config.safety.llheartbeat.observe_jvel;
    send.config.safety.llheartbeat.observe_jtor = config.safety.llheartbeat.observe_jtor;
    send.config.safety.llheartbeat.enable_IMU = config.safety.llheartbeat.enable_IMU;
    send.config.safety.llheartbeat.IMU_timeout = config.safety.llheartbeat.IMU_timeout;
    send.config.safety.llheartbeat.observe_accel = config.safety.llheartbeat.observe_accel;
    send.config.safety.llheartbeat.observe_gyro = config.safety.llheartbeat.observe_gyro;

    send.config.safety.torso.enabled = config.safety.torso.enabled;
    send.config.safety.torso.observe_roll = config.safety.torso.observe_roll;
    send.config.safety.torso.roll_limit = config.safety.torso.roll_limit;
    send.config.safety.torso.observe_pitch = config.safety.torso.observe_pitch;
    send.config.safety.torso.pitch_limit = config.safety.torso.pitch_limit;
    send.config.safety.torso.observe_gyro = config.safety.torso.observe_gyro;
    send.config.safety.torso.gyro_limit = config.safety.torso.gyro_limit;
}

/**
 * @brief Set state from local buffer
 *
 */
void Robot::dataAdaptor_(ClientNetworkConfig::AlliedState &recv, AlliedDataTypes::State &state)
{
    for(int i = 0; i < NDOF; i++)
    {
        state.joint.pos[i] = recv.state.joint.pos[i];
        state.joint.vel[i] = recv.state.joint.vel[i];
        state.joint.tor[i] = recv.state.joint.tor[i];
        state.joint.kp[i] = recv.state.joint.kp[i];
        state.joint.kd[i] = recv.state.joint.kd[i];
    }

    for(int i = 0; i < 6; i++)
    {
        state.torso.pos[i] = recv.state.torso.pos[i];
        state.torso.vel[i] = recv.state.torso.vel[i];
    }

    for(int i = 0; i < 3; i++)
    {
        state.imu.acc[i] = recv.state.imu.acc[i];
        state.imu.gyro[i] = recv.state.imu.gyro[i];
        state.imu.mag[i] = recv.state.imu.mag[i];
        state.imu.euler[i] = recv.state.imu.gyro[i];
    }
}

/**
 * @brief Set config from local buffer
 *
 */
void Robot::dataAdaptor_(ClientNetworkConfig::AlliedState &recv, QuadDataTypes::CONFIG_SET &config)
{
    config.master.shutdown = recv.config.master.shutdown;
    config.master.exit = recv.config.master.exit;
    config.master.killed = recv.config.master.killed;

    config.motion.planner = static_cast<MotionDataTypes::TaskTypes>(recv.config.motion.planner);
    config.motion.sequence.type = static_cast<GestureTypes>(recv.config.motion.sequence.type);
    config.motion.sequence.seq = static_cast<int>(recv.config.motion.sequence.seq);
    config.motion.strategy.type = static_cast<MotionModes>(recv.config.motion.strategy.type);
    config.motion.strategy.seq = static_cast<MotionGaits>(recv.config.motion.strategy.seq);

    config.safety.llsafety.enabled = recv.config.safety.llsafety.enabled;
    config.safety.llsafety.jpos = recv.config.safety.llsafety.jpos;
    config.safety.llsafety.jvel = recv.config.safety.llsafety.jvel;
    config.safety.llsafety.jtor = recv.config.safety.llsafety.jtor;
    config.safety.llsafety.dtor = recv.config.safety.llsafety.dtor;

    for(int i = 0; i < NDOF; i++)
    {
        config.safety.llsafety.lim_jpos_min[i] = recv.config.safety.llsafety.lim_jpos_min[i];
        config.safety.llsafety.lim_jpos_max[i] = recv.config.safety.llsafety.lim_jpos_max[i];
        config.safety.llsafety.lim_jvel[i] = recv.config.safety.llsafety.lim_jvel[i];
        config.safety.llsafety.lim_jtor[i] = recv.config.safety.llsafety.lim_jtor[i];
        config.safety.llsafety.lim_dtor[i] = recv.config.safety.llsafety.lim_dtor[i];
    }

    config.safety.llheartbeat.enabled = recv.config.safety.llheartbeat.enabled;
    config.safety.llheartbeat.init_disable_time = recv.config.safety.llheartbeat.init_disable_time;
    config.safety.llheartbeat.enable_CAN = recv.config.safety.llheartbeat.enable_CAN;
    config.safety.llheartbeat.CAN_timeout = recv.config.safety.llheartbeat.CAN_timeout;
    config.safety.llheartbeat.observe_jpos = recv.config.safety.llheartbeat.observe_jpos;
    config.safety.llheartbeat.observe_jvel = recv.config.safety.llheartbeat.observe_jvel;
    config.safety.llheartbeat.observe_jtor = recv.config.safety.llheartbeat.observe_jtor;
    config.safety.llheartbeat.enable_IMU = recv.config.safety.llheartbeat.enable_IMU;
    config.safety.llheartbeat.IMU_timeout = recv.config.safety.llheartbeat.IMU_timeout;
    config.safety.llheartbeat.observe_accel = recv.config.safety.llheartbeat.observe_accel;
    config.safety.llheartbeat.observe_gyro = recv.config.safety.llheartbeat.observe_gyro;

    config.safety.torso.enabled = recv.config.safety.torso.enabled;
    config.safety.torso.observe_roll = recv.config.safety.torso.observe_roll;
    config.safety.torso.roll_limit = recv.config.safety.torso.roll_limit;
    config.safety.torso.observe_pitch = recv.config.safety.torso.observe_pitch;
    config.safety.torso.pitch_limit = recv.config.safety.torso.pitch_limit;
    config.safety.torso.observe_gyro = recv.config.safety.torso.observe_gyro;
    config.safety.torso.gyro_limit = recv.config.safety.torso.gyro_limit;
}
