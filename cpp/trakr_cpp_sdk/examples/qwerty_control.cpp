#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#include <cstdlib>

#include "robot.h"

#define FREQ 400

#define ROBOT_IP "192.168.3.50"
#define ROBOT_PORT 15251

#define LINEAR_VEL_STEP  0.1f
#define ANGULAR_VEL_STEP 0.15f

static struct termios g_orig_termios;
static std::atomic_bool g_term_setup{false};

static void restore_terminal() {
    if (g_term_setup.load()) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_term_setup.store(false);
    }
}

static void set_raw_terminal() {
    if (g_term_setup.load()) return;
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) == -1) {
        perror("tcgetattr");
        return;
    }
    atexit(restore_terminal);

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) {
        perror("tcsetattr");
        return;
    }

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    g_term_setup.store(true);
}

static bool kbhit(char &out) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv{0, 0};
    if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0 &&
        FD_ISSET(STDIN_FILENO, &fds)) {
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            out = c;
            return true;
        }
    }
    return false;
}

// vel[0..2] = angular (x, y, z), vel[3..5] = linear (x, y, z)
// forward = y (vel[4]), left = x (vel[3]), yaw = z (vel[2])
float vel[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
std::mutex mut_;
std::atomic_bool running_{true};
bool set_config = false;
QuadDataTypes::CONFIG_SET config;
QuadDataTypes::CONFIG_SET pending_config;

static void print_status() {
    std::cout << "\r[Vel] Linear Y: " << std::fixed << std::setprecision(2) << vel[4]
              << " | Angular Z: " << vel[2] << "        " << std::flush;
}

static void print_controls() {
    std::cout << "\n=== QWERTY Teleop Control ===" << std::endl;
    std::cout << "W/S   : Forward/Backward velocity (step: " << LINEAR_VEL_STEP << ")" << std::endl;
    std::cout << "A/D   : Rotate Left/Right (step: " << ANGULAR_VEL_STEP << ")" << std::endl;
    std::cout << "SPACE : Stop all motion" << std::endl;
    std::cout << "1     : Gesture orientation mode" << std::endl;
    std::cout << "2     : Classical stance mode" << std::endl;
    std::cout << "3     : AI motion mode" << std::endl;
    std::cout << "4     : Developer motion mode" << std::endl;
    std::cout << "5     : Left shake hand" << std::endl;
    std::cout << "6     : Right shake hand" << std::endl;
    std::cout << "9     : Exit robot" << std::endl;
    std::cout << "0     : Shutdown robot" << std::endl;
    std::cout << "ESC   : Quit program" << std::endl;
    std::cout << "==============================\n" << std::endl;
}

void input_loop() {
    while (running_.load()) {
        char c;
        if (kbhit(c)) {
            std::lock_guard<std::mutex> lock(mut_);

            switch (c) {
                case 'w':
                case 'W':
                    vel[4] += LINEAR_VEL_STEP;
                    vel[2] = 0.f;
                    print_status();
                    break;

                case 's':
                case 'S':
                    vel[4] -= LINEAR_VEL_STEP;
                    vel[2] = 0.f;
                    print_status();
                    break;

                case 'a':
                case 'A':
                    vel[2] += ANGULAR_VEL_STEP;
                    print_status();
                    break;

                case 'd':
                case 'D':
                    vel[2] -= ANGULAR_VEL_STEP;
                    print_status();
                    break;

                case ' ':
                    for (int i = 0; i < 6; i++) vel[i] = 0.f;
                    print_status();
                    break;

                case '1':
                    pending_config.motion.planner = MotionDataTypes::TaskTypes::eGesture;
                    pending_config.motion.sequence.type = GestureTypes::eOrientation;
                    set_config = true;
                    std::cout << "\nMode: Gesture Orientation" << std::endl;
                    break;

                case '2':
                    pending_config.motion.planner = MotionDataTypes::TaskTypes::eMotion;
                    pending_config.motion.strategy.type = MotionModes::eClassicalMode;
                    pending_config.motion.strategy.seq = MotionGaits::eStance;
                    set_config = true;
                    std::cout << "\nMode: Classical Stance" << std::endl;
                    break;

                case '3':
                    pending_config.motion.planner = MotionDataTypes::TaskTypes::eMotion;
                    pending_config.motion.strategy.type = MotionModes::eAIMode;
                    set_config = true;
                    std::cout << "\nMode: AI Motion" << std::endl;
                    break;

                case '4':
                    pending_config.motion.planner = MotionDataTypes::TaskTypes::eMotion;
                    pending_config.motion.strategy.type = MotionModes::eDeveloperMode;
                    set_config = true;
                    std::cout << "\nMode: Developer" << std::endl;
                    break;

                case '5':
                    pending_config.motion.planner = MotionDataTypes::TaskTypes::eGesture;
                    pending_config.motion.sequence.type = GestureTypes::eLeftShakeHand;
                    set_config = true;
                    std::cout << "\nMode: Left Shake Hand" << std::endl;
                    break;

                case '6':
                    pending_config.motion.planner = MotionDataTypes::TaskTypes::eGesture;
                    pending_config.motion.sequence.type = GestureTypes::eRightShakeHand;
                    set_config = true;
                    std::cout << "\nMode: Right Shake Hand" << std::endl;
                    break;

                case '9':
                    pending_config.master.doExit();
                    set_config = true;
                    std::cout << "\nExiting robot..." << std::endl;
                    break;

                case '0':
                    pending_config.master.doShutDown();
                    set_config = true;
                    std::cout << "\nShutting down robot..." << std::endl;
                    break;

                case 27: // ESC
                    running_.store(false);
                    std::cout << "\nQuitting..." << std::endl;
                    break;

                default:
                    break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

int main()
{
    unsigned long int sleep_ns = 1000000000 / FREQ;

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

    if (!robot.setup(config, plan)) {
        std::cout << "Failed to setup robot" << std::endl;
        return 1;
    }

    robot.getConfig(config);

    set_raw_terminal();
    print_controls();

    std::thread input_thread(&input_loop);

    while (robot.isAlive() && running_.load()) {
        auto start = std::chrono::high_resolution_clock::now();

        if (!robot.run()) {
            break;
        }

        mut_.lock();
        robot.getData(state);
        robot.getConfig(config);
        robot.getConfigStatus();

        if (config.motion.planner == MotionDataTypes::TaskTypes::eMotion &&
            config.motion.strategy.type == MotionModes::eClassicalMode) {
            float total_vel = std::abs(vel[2]) + std::abs(vel[3]) + std::abs(vel[4]);
            if (total_vel < 0.01f) {
                config.motion.strategy.seq = MotionGaits::eStance;
            } else {
                config.motion.strategy.seq = MotionGaits::eTrot;
            }
            set_config = true;
        }

        for (int i = 0; i < 6; i++) {
            plan.torso.vel[i] = vel[i];
        }

        if (set_config) {
            config = pending_config;
            std::cout << "Setting config" << std::endl;
            robot.setConfig(config);
            set_config = false;
        }
        robot.setData(plan);
        mut_.unlock();

        while (std::chrono::duration_cast<std::chrono::duration<double, std::nano>>(
                   std::chrono::high_resolution_clock::now() - start).count() < sleep_ns) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }

    running_.store(false);

    if (input_thread.joinable()) {
        input_thread.join();
    }

    restore_terminal();
    std::cout << "\nProgram terminated." << std::endl;

    return 0;
}
