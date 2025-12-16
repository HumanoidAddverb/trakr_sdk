#include <chrono>
#include <thread>
#include <iostream>

#include "robot.h"

#define FREQ 400

#define ROBOT_IP "192.168.3.50"
#define ROBOT_PORT 15251

int main()
{
    unsigned long int sleep_ns_ = 1000000000/FREQ;

    Robot robot(ROBOT_IP, ROBOT_PORT);

    AlliedDataTypes::Plan plan;
    AlliedDataTypes::State state;
    QuadDataTypes::CONFIG_SET config;

    plan.joint.kp = JointVector::Zero();
    plan.joint.kd = JointVector::Zero();
    plan.joint.pos = JointVector::Zero();
    plan.joint.vel = JointVector::Zero();
    plan.joint.tor = JointVector::Zero();

    plan.torso.pos = Vector6::Zero();
    plan.torso.vel = Vector6::Zero();

    robot.setup(config, plan);

    robot.getConfig(config);

    bool config_status;

    bool set_config = false;

    while(robot.isAlive())
    {
        auto start = std::chrono::high_resolution_clock::now();

        if(!robot.run())
        {
            break;
        }

        robot.getData(state);
        robot.getConfig(config);

        // this is feedback for last config set.
        // 0 . no config change was requested
        // 1 . Accepted
        // 2 . Rejected 
        config_status = robot.getConfigStatus();

        // do something here
        std::cout << "IMU Acc: " << state.imu.acc.transpose() << std::endl;

        if(set_config)
        {
            robot.setConfig(config);
            set_config = false;
        }
        robot.setData(plan);

        while(std::chrono::duration_cast<std::chrono::duration<double, std::nano>>(std::chrono::high_resolution_clock::now() - start).count() < sleep_ns_)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }

    return 0;
}