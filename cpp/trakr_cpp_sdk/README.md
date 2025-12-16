# trakr_cpp_sdk C++ CMake Package
A minimal package that allows communicating with Trakr over a TCP/IP socket.
Performs minimal (and mandatory) sanity checks over the data before creating and sending packets to robot.

The package requires specific OS/Architecture to be installed,
 - Tested with **Ubuntu 22.04** and **Ubuntu 20.04**
 - **x86-64 Architecture** (ARM support may be added if needed)

And, has the following dependencies,
- **build-essential** : Install with `sudo apt install build-essential`
- **cmake** : Install with `sudo apt install cmake`
- **libeigen3** : Install with `sudo apt install libeigen3-dev`

This repository includes the sources for trakr_cpp_sdk CMake package, which can be used as a workspace, and can be installed as a CMake package and included in other CMake workspaces with `find_package(trakr_cpp_sdk)`. \
For usage methods for both, see instructions below.

## Usage
The CMake package can either be built as a workspace, and can be installed as a CMake package.

### 1. Using as CMake Workspace
To build the code with this directory as workspace,
```
mkdir build
cd build
cmake ..
make -j8
```
This, by default, builds the *trakr_cpp_sdk* along with the examples (under `bin/`) within this package. \
Examples included and built with the code can be run as,
```
./bin/basic
```
or
```
./bin/keyboard
```

> NOTE: The examples are built with the default IP address of Trakr (192.168.3.50). To change this address (if required), edit the *cpp* for examples under `examples/` directory, and re-build the code.

### 2. Installing as CMake Package
> NOTE: Use this mode if you wish to integrate this SDK within your C++ workspace.

To install the package as a CMake package that can be included in any workspace, \
first *follow the instructions in (1.)*, and then run the command (within the `build/` directory),
```
sudo make install -j
```

This installs the `trakr_cpp_sdk` as a library under `/usr/include` and `/usr/lib` directories. \
To change the installation prefix (for eg., `/usr/local/`), edit the `CMAKE_PREFIX_PATH` in `CMakeLists.txt`.

## Usage

To test whether the package is correctly built, try running an example code,
```
./bin/basic
```
If the robot is available, the Robot will start bring-up sequence and the terminal output will look something like,
```
[NETWORKMAN] TCPClient Communication Setup
[CONNECTIONMAN] Peer was not connected. Will try to connect to client
[SOCKETHANDLER] Got Data
[NETWORKMAN] Socket connected and data availability checked.
[ROBOT] Connected!
[ROBOT] Waiting for Robot to BringUp!
[ROBOT] Robot BringUp Completed!
```

### A. Initializing Data Buffers
The entire communication with the robot consists of three data types (see `docs/DataTypes.md`):
#### 1. `AlliedDataTypes::Plan`
Consists of the continuous high-level and low-level control commands sent from Client to Robot

#### 2. `AlliedDataTypes::State`
Consists of the continuous feedback from Robot to Client. (State Estimation, Joint States, etc.)

#### 3. `QuadDataTypes::CONFIG_SET`
This holds the configuration of robot defining the type of tasks and safeties to be performed, and is used in two-ways,
- Sent by Client to Robot to set the tasks to be performed (for eg. Left Hand Shake)
- Sent by Robot to Client to indicate the latest configuration present in the Robot.

These data types can be imported as,
```
#include "robot.h"

AlliedDataTypes::Plan plan;
AlliedDataTypes::State state;
QuadDataTypes::CONFIG_SET config;
```

### B. Setting-Up Connection 
The entire communications with robot have been wrapped under `Robot` class. \
We set-up an object with the IP Address and Port of Robot,
```
#define ROBOT_IP "192.168.3.50"
#define ROBOT_PORT 15251

Robot robot(ROBOT_IP, ROBOT_PORT);
```

### C. Starting Communications
To initialize a connection, we will need some data that we can send over the socket to perform handshake.
\
The `plan` and `config` buffers created above, hold the default and safe values for commands and configuration(s) of Robot. So, we intialize a connection with the buffers as,
```
plan.joint.kp = JointVector::Zero();
plan.joint.kd = JointVector::Zero();
plan.joint.pos = JointVector::Zero();
plan.joint.vel = JointVector::Zero();
plan.joint.tor = JointVector::Zero();

plan.torso.pos = Vector6::Zero();
plan.torso.vel = Vector6::Zero();

robot.setup(config, plan); // -> returns a bool indicating success or failure
```
This function call is a **blocking** call. This means,
- Robot, before it starts accepting user commands and configurations, needs to perform a "Bring-Up" sequence (perform a stand-up and some necessary computations).
- When the client initializes a connection through `robot.setup()`, the code **blocks** until the Robot confirms that the "Bring-Up" sequence is completed.

The call returns a *bool* value, indicating whether the robot has successfully been initialized, and can *fail* in case of any hardware issues that prevent the robot from start-up (to protect the hardware)

### D. Starting Continuous Data-Stream
Now that our connection has been initialized, we start a continuous `while` loop that continuously perform the data-exchange between client and robot. This happens through a single function call,
```
robot.run(); // -> returns bool indicating if socket communication is successful
```
Note that this call does not require any arguments. Calling this function sends the last-set commands to robot, and receives the latest feedback from Robot, storing all the data in internal buffers. To access and write to the buffers, we have different calls as,
```
robot.getData(state); // -> populates the RobotFeedback into "state" buffer
robot.getConfig(config); // -> populates the latest configuration of Robot into "config" buffer

robot.setConfig(config); // -> populates the latest configuration from Client into the Robot.
robot.setData(plan); // -> populates the latest commands from Client into the Robot.
```
Not every configuration sent by client will be a *valid* configuration. This is because what is possible to be done at some point in time, may not be possible at a different instance. To check whether the configuration sent to Robot is *valid*, we can call the `getConfigStatus` method as, 
```
robot.getConfigStatus(); // -> returns "int",
// 0 -> No configuration was sent to Robot
// 1 -> Configuration was Accepted
// 2 -> Configuration was Rejected
```

And since we'll only be communicating if the robot is "alive" (communications are up), we can check that through,
```
robot.isAlive(); // -> returns bool indicating whether communications/robot is/are still alive.
```

> **Note:** The configuration of Robot ("config") is something that is set *occasionally* rather than *continuously*. \
This means that the configuration should be updated *only when there is a change in configuration*. Since the configuration ("CONFIG_SET") is a vast set of different settings, Robot updates the configuration only when User/Client **explicitely** tells the robot to set a configuration, through the `robot.setConfig(config)` call. \
**Call `robot.setConfig(config)` only when needed**. Continously updating configuration will cause the robot to re-apply an already-applied configuration infinite number of times, which can cause *undefined* behavior.

To set the configuration when required, we use a `bool set_config` to store and check whether a configuration has to be applied, and the code will look something like,
```
...
set_config = true; // somewhere in client's code
...

if(set_config)
{
    robot.setConfig(config);
    set_config = false;
}
```

### E. Minimal Example Code
Bringing everything together, a basic code to communicate with Trakr, will look something like, \
**See: `cpp/trakr_cpp_sdk/examples/basic.cpp`**
```
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
        // read "state" and finally set "plan" and if required, set "config"

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
```
> **Note**: As a general practice, ensure that these method calls happen in the order shown in this example code. This doesn't mean that this is the *only* correct way, but rather that this is a *tried and tested* way that will always work, out of many possible ways that there may be.

### F. Example Implementation(s)
A detailed and tested implementation for this SDK is given in **`cpp/trakr_cpp_sdk/examples/keyboard.cpp`**, that allows controlling all available features of Robot through teleoperation with keyboard.

## Best Practices
The `while` loop, considering that the latest data should reach Robot and should be fetched from Robot, must be continuously running at all times. This means,
- Do not use any blocking method calls inside this while loop. If any computation from client requires such a method call, consider creating a different thread (either for Robot or Client's Blocking Call).
- The while loop should be running at ~400Hz. A slower loop *might* cause less-optimal behavior.

> Additional point to note is that since the SDK maintains its own buffers to send and receive the packets, the packets are sent-and-received in a *different thread*. This thread is created and started (after `setup()`) inside the SDK itself, and runs at a high frequency.

## Documentation

See further documentation in `docs` directory, for details regarding different modes of operation, how to switch between them, etc.