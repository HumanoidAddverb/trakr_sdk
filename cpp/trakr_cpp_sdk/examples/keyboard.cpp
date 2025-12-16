#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>

#include "robot.h"

#define FREQ 400

#define ROBOT_IP "192.168.3.50"
#define ROBOT_PORT 15251

#define FORWARD_VEL 0.3
#define FORWARD_RUN 0.6
#define FORWARD_DASH 0.8
#define FORWARD_NEXT 1.0
#define BACKWARD_VEL -0.3
#define TROT_VEL 0.02
#define RIGHT_VEL 0.3
#define LEFT_VEL -0.3
#define SMALL_TURN_ACK 0.05
#define SMALL_TURN_CK -0.05
#define BIG_TURN_ACK 0.5
#define BIG_TURN_CK -0.5
#define TRIAL_VEL 0.001
#define LOOK_UP 0.1
#define LOOK_UP_BIG 0.15
#define LOOK_NET 0.1


float vel[6] = {0., 0., 0., 0., 0., 0.};
char varbuf_[10];
int var_;
std::mutex mut_;
bool changed = false;
bool set_config = false;
bool running_ = true;
QuadDataTypes::CONFIG_SET config;


void userInput()
{
    while(running_)
    {
        std::cin >> varbuf_;

        mut_.lock();
        if(sscanf(varbuf_, "%d", &var_) != 1){
            var_ = 0;
        }

        changed = true;
        mut_.unlock();
    }
}

void doAction()
{
    switch (var_)
    {
    case 0:
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = 0.;
        break;

    case 1:
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = FORWARD_VEL;
        break;

    case 2:
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = BACKWARD_VEL;
        break;

    case 3:
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = TROT_VEL;
        break;

    case 4:
        vel[2] = 0.;
        vel[3] = RIGHT_VEL;
        vel[4] = 0.;
        break;

    case 5:
        vel[2] = 0.;
        vel[3] = LEFT_VEL;
        vel[4] = 0.;
        break;

    case 6:
        vel[2] = SMALL_TURN_ACK;
        break;

    case 7:
        vel[2] = SMALL_TURN_CK;
        break;

    case 8:
        vel[2] = BIG_TURN_ACK;
        vel[3] = 0.;
        vel[4] = 0.;
        break;

    case 9:
        vel[2] = BIG_TURN_CK;
        vel[3] = 0.;
        vel[4] = 0.;
        break;

    case 31:
        config.motion.planner = MotionDataTypes::TaskTypes::eGesture;
        config.motion.sequence.type = GestureTypes::eOrientation;
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 32:
        config.motion.planner = MotionDataTypes::TaskTypes::eMotion;
        config.motion.strategy.type = MotionModes::eClassicalMode;
        config.motion.strategy.seq = MotionGaits::eStance;
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 33:
        config.motion.planner = MotionDataTypes::TaskTypes::eMotion;
        config.motion.strategy.type = MotionModes::eAIMode;
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 34:
        config.motion.planner = MotionDataTypes::TaskTypes::eMotion;
        config.motion.strategy.type = MotionModes::eDeveloperMode;
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 39:
        config.master.doExit();
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 41:
        config.motion.planner = MotionDataTypes::TaskTypes::eGesture;
        config.motion.sequence.type = GestureTypes::eLeftShakeHand;
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 42:
        config.motion.planner = MotionDataTypes::TaskTypes::eGesture;
        config.motion.sequence.type = GestureTypes::eRightShakeHand;
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 90:
        config.master.doShutDown();
        if(changed)
        {
            set_config = true;
            changed = false;
        }
        break;

    case 93:
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = FORWARD_RUN;
        break;

    case 94:
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = FORWARD_DASH;
        break;

    case 95 : 
        vel[2] = 0.;
        vel[3] = 0.;
        vel[4] = FORWARD_NEXT;
        break;

    default:
        break;
    }
    // changed = false;

    if(config.motion.planner == MotionDataTypes::TaskTypes::eMotion)
    {
        if(config.motion.strategy.type == MotionModes::eClassicalMode)
        {
            if((std::abs(vel[2]) + std::abs(vel[3]) + std::abs(vel[4])) < 0.01)
            {
                config.motion.strategy.seq = MotionGaits::eStance;
                if(changed)
                {
                    set_config = true;
                    changed = false;
                }
            }
            else
            {
                config.motion.strategy.seq = MotionGaits::eTrot;
                if(changed)
                {
                    set_config = true;
                    changed = false;
                }
            }
        }
    }
}

int main()
{
    unsigned long int sleep_ns_ = 1000000000/FREQ;

    Robot robot(ROBOT_IP, ROBOT_PORT);

    AlliedDataTypes::Plan plan;
    AlliedDataTypes::State state;

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

    bool config_status;

    // bool set_config = false;
    
    std::thread input_ = std::thread(&userInput);

    while(robot.isAlive())
    {
        auto start = std::chrono::high_resolution_clock::now();

        if(!robot.run())
        {
            break;
        }

        mut_.lock();
        robot.getData(state);
        robot.getConfig(config);

        // this is feedback for last config set.
        // 0 . no config change was requested
        // 1 . Accepted
        // 2 . Rejected 
        config_status = robot.getConfigStatus();

        // do user actions
        doAction();
        for(int i = 0; i < 6; i++)
        {
            plan.torso.vel[i] = vel[i];
        }

        if(set_config)
        {
            robot.setConfig(config);
            set_config = false;
        }
        robot.setData(plan);
        mut_.unlock();

        while(std::chrono::duration_cast<std::chrono::duration<double, std::nano>>(std::chrono::high_resolution_clock::now() - start).count() < sleep_ns_)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }

    running_ = false;
    
    if(input_.joinable())
    {
        input_.detach();
    }

    return 0;
}
