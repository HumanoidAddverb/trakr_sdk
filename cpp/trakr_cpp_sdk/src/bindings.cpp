#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "trakr_sdk.h"

namespace py = pybind11;

PYBIND11_MODULE(trakr_python_sdk, m)
{
    m.doc() = "Trakr Socket SDK";

    py::class_<SocketDataTypes::JointState>(m, "JointState")
        .def(py::init())
        .def_readwrite("pos", &SocketDataTypes::JointState::pos)
        .def_readwrite("vel", &SocketDataTypes::JointState::vel)
        .def_readwrite("tor", &SocketDataTypes::JointState::tor)
        .def_readwrite("kp", &SocketDataTypes::JointState::kp)
        .def_readwrite("kd", &SocketDataTypes::JointState::kd)
        .def_readwrite("timestamp", &SocketDataTypes::JointState::timestamp);

    py::class_<SocketDataTypes::TorsoState>(m, "TorsoState")
        .def(py::init())
        .def_readwrite("pos", &SocketDataTypes::TorsoState::pos)
        .def_readwrite("vel", &SocketDataTypes::TorsoState::vel)
        .def_readwrite("timestamp", &SocketDataTypes::TorsoState::timestamp);

    py::class_<SocketDataTypes::IMUState>(m, "IMUState")
        .def(py::init())
        .def_readwrite("acc", &SocketDataTypes::IMUState::acc)
        .def_readwrite("gyro", &SocketDataTypes::IMUState::gyro)
        .def_readwrite("mag", &SocketDataTypes::IMUState::mag)
        .def_readwrite("euler", &SocketDataTypes::IMUState::euler)
        .def_readwrite("timestamp", &SocketDataTypes::IMUState::timestamp);

    py::class_<SocketDataTypes::BatteryState>(m, "BatteryState")
        .def(py::init());

    py::class_<SocketDataTypes::Plan>(m, "Plan")
        .def(py::init())
        .def_readwrite("torso", &SocketDataTypes::Plan::torso)
        .def_readwrite("joint", &SocketDataTypes::Plan::joint);

    py::class_<SocketDataTypes::State>(m, "State")
        .def(py::init())
        .def_readwrite("torso", &SocketDataTypes::State::torso)
        .def_readwrite("joint", &SocketDataTypes::State::joint)
        .def_readwrite("imu", &SocketDataTypes::State::imu)
        .def_readwrite("power", &SocketDataTypes::State::power);


    // Binding Configs for Trakr
    py::class_<SocketDataTypes::RobotConfig>(m, "RobotConfig")
        .def(py::init())
        .def_readwrite("shutdown", &SocketDataTypes::RobotConfig::shutdown)
        .def_readwrite("exit", &SocketDataTypes::RobotConfig::exit)
        .def_readwrite("killed", &SocketDataTypes::RobotConfig::killed);
    
    // Starting with MotionConfigs
    py::class_<SocketDataTypes::MotionBaseConfig>(m, "MotionBaseConfig")
        .def(py::init())
        .def_readwrite("type", &SocketDataTypes::MotionBaseConfig::type)
        .def_readwrite("seq", &SocketDataTypes::MotionBaseConfig::seq);

    py::class_<SocketDataTypes::MotionConfig>(m, "MotionConfig")
        .def(py::init())
        .def_readwrite("planner", &SocketDataTypes::MotionConfig::planner)
        .def_readwrite("sequence", &SocketDataTypes::MotionConfig::sequence)
        .def_readwrite("strategy", &SocketDataTypes::MotionConfig::strategy);

    // SafetyConfigs
    py::class_<SocketDataTypes::SafetyBaseConfig>(m, "SafetyBaseConfig")
        .def(py::init())
        .def_readwrite("enabled", &SocketDataTypes::SafetyBaseConfig::enabled);

    py::class_<SocketDataTypes::LLSafetyConfig>(m, "LLSafetyConfig")
        .def(py::init())
        .def_readwrite("enabled", &SocketDataTypes::LLSafetyConfig::enabled)
        .def_readwrite("jpos", &SocketDataTypes::LLSafetyConfig::jpos)
        .def_readwrite("jvel", &SocketDataTypes::LLSafetyConfig::jvel)
        .def_readwrite("jtor", &SocketDataTypes::LLSafetyConfig::jtor)
        .def_readwrite("dtor", &SocketDataTypes::LLSafetyConfig::dtor)
        .def_readwrite("lim_jpos_min", &SocketDataTypes::LLSafetyConfig::lim_jpos_min)
        .def_readwrite("lim_jpos_max", &SocketDataTypes::LLSafetyConfig::lim_jpos_max)
        .def_readwrite("lim_jvel", &SocketDataTypes::LLSafetyConfig::lim_jvel)
        .def_readwrite("lim_jtor", &SocketDataTypes::LLSafetyConfig::lim_jtor)
        .def_readwrite("lim_dtor", &SocketDataTypes::LLSafetyConfig::lim_dtor);

    py::class_<SocketDataTypes::LLHeartbeatConfig>(m, "LLHeartbeatConfig")
        .def(py::init())
        .def_readwrite("enabled", &SocketDataTypes::LLHeartbeatConfig::enabled)
        .def_readwrite("init_disable_time", &SocketDataTypes::LLHeartbeatConfig::init_disable_time)
        .def_readwrite("enable_CAN", &SocketDataTypes::LLHeartbeatConfig::enable_CAN)
        .def_readwrite("CAN_timeout", &SocketDataTypes::LLHeartbeatConfig::CAN_timeout)
        .def_readwrite("observe_jpos", &SocketDataTypes::LLHeartbeatConfig::observe_jpos)
        .def_readwrite("observe_jvel", &SocketDataTypes::LLHeartbeatConfig::observe_jvel)
        .def_readwrite("observe_jtor", &SocketDataTypes::LLHeartbeatConfig::observe_jtor)
        .def_readwrite("enable_IMU", &SocketDataTypes::LLHeartbeatConfig::enable_IMU)
        .def_readwrite("IMU_timeout", &SocketDataTypes::LLHeartbeatConfig::IMU_timeout)
        .def_readwrite("observe_accel", &SocketDataTypes::LLHeartbeatConfig::observe_accel)
        .def_readwrite("observe_gyro", &SocketDataTypes::LLHeartbeatConfig::observe_gyro);

    py::class_<SocketDataTypes::TorsoSafetyConfig>(m, "TorsoSafetyConfig")
        .def(py::init())
        .def_readwrite("enabled", &SocketDataTypes::TorsoSafetyConfig::enabled)
        .def_readwrite("observe_roll", &SocketDataTypes::TorsoSafetyConfig::observe_roll)
        .def_readwrite("roll_limit", &SocketDataTypes::TorsoSafetyConfig::roll_limit)
        .def_readwrite("observe_pitch", &SocketDataTypes::TorsoSafetyConfig::observe_pitch)
        .def_readwrite("pitch_limit", &SocketDataTypes::TorsoSafetyConfig::pitch_limit)
        .def_readwrite("observe_gyro", &SocketDataTypes::TorsoSafetyConfig::observe_gyro)
        .def_readwrite("gyro_limit", &SocketDataTypes::TorsoSafetyConfig::gyro_limit);

    py::class_<SocketDataTypes::SafetyConfig>(m, "SafetyConfig")
        .def(py::init())
        .def_readwrite("llsafety", &SocketDataTypes::SafetyConfig::llsafety)
        .def_readwrite("llheartbeat", &SocketDataTypes::SafetyConfig::llheartbeat)
        .def_readwrite("torso", &SocketDataTypes::SafetyConfig::torso);

    py::class_<SocketDataTypes::Config>(m, "Config")
        .def(py::init())
        .def_readwrite("status", &SocketDataTypes::Config::status)
        .def_readwrite("master", &SocketDataTypes::Config::master)
        .def_readwrite("motion", &SocketDataTypes::Config::motion)
        .def_readwrite("safety", &SocketDataTypes::Config::safety);

    // Binding AlliedData (Plan + Config) and (State + Config)
    py::class_<SocketDataTypes::AlliedPlan>(m, "AlliedPlan")
        .def(py::init())
        .def_readwrite("plan", &SocketDataTypes::AlliedPlan::plan)
        .def_readwrite("config", &SocketDataTypes::AlliedPlan::config);

    py::class_<SocketDataTypes::AlliedState>(m, "AlliedState")
        .def(py::init())
        .def_readwrite("state", &SocketDataTypes::AlliedState::state)
        .def_readwrite("config", &SocketDataTypes::AlliedState::config);

    // Binding the main wrapper over our network implementation
    py::class_<ClientNetworkConfig::RobotNetwork>(m, "RobotNetwork")
        .def(py::init())
        .def("setup", &ClientNetworkConfig::RobotNetwork::setup)
        .def("connect", &ClientNetworkConfig::RobotNetwork::connect)
        .def("disconnect", &ClientNetworkConfig::RobotNetwork::disconnect)
        .def("isConnected", &ClientNetworkConfig::RobotNetwork::isConnected)
        .def("shutdown", &ClientNetworkConfig::RobotNetwork::shutdown)
        .def("readData", &ClientNetworkConfig::RobotNetwork::readData)
        .def("getData", &ClientNetworkConfig::RobotNetwork::getData)
        .def("setData", &ClientNetworkConfig::RobotNetwork::setData)
        .def("writeData", &ClientNetworkConfig::RobotNetwork::writeData);
}