#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>

#include "robot.h"

// #include <torch/torch.h>
// #include <torch/script.h>

#define FREQ 400

#define ROBOT_IP "192.168.3.50"
#define ROBOT_PORT 15251


void doInference()
{

}

void doAction()
{
    doInference();
}

int main()
{
    // torch::set_num_threads(1);
    // torch::set_num_interop_threads(1);

    unsigned long int sleep_ns_ = 1000000000/FREQ;

    Robot robot(ROBOT_IP, ROBOT_PORT, LOW_LEVEL);

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

    if(!robot.setup(config, plan))
    {
        std::cout << "[MAIN] Failed to setup robot" << std::endl;
    }

    robot.getConfig(config);

    bool config_status, set_config;

    auto begin = std::chrono::high_resolution_clock::now();

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
        // 1 . Requested
        // 2 . Accepted
        // 3 . Rejected 
        config_status = robot.getConfigStatus();

        // do user actions
        doAction();

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
