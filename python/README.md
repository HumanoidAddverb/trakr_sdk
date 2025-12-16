# trakr_sdk Python Package
A minimal package that allows communicating with Trakr over a TCP/IP socket.
Performs minimal (and mandatory) sanity checks over the data before creating and sending packets to robot.

The package requires specific OS/Architecture to be installed,
 - **Ubuntu 20.04** (or equivalent linux flavor)
 - **x86-64 Architecture** (ARM support may be added if needed)
 - **Python == 3.8** (Other versions can be supported if needed)

And, has the following dependencies,
- **numpy >= 1.19** 

This repository includes the sources for SDK, along with the packaged Python Wheel(s) (in GitHub Releases). For installation methods for both, see instructions below.

## Installation
There are two ways to install and use the package, depending on your requirements as a developer.
> We assume that Python3.8 is installed. If not, create a separate Conda environment with Python3.8 as, \
`conda create --name trakr python=3.8` && `conda activate trakr`

### 1. Out-of-the-Box Installation (Recommended)
To use the SDK without modifications, the latest _wheel_ from _releases_ can be installed as,
```
python3.8 -m pip install "https://github.com/HumanoidAddverb/trakr_sdk/releases/download/v1.0.0/trakr_sdk-1.0.0-cp38-cp38-linux_x86_64.whl"
```
This installs the `trakr_sdk` package and can be imported in any code running in the same environment. 
Supports/allows  code-completion in compatible IDE(s).

### 2. Experimental Installation (Advanced)
> NOTE: Use this mode only and only if you are completely understand the consequences of your changes.

To perform additions to the _trakr_sdk_ itself (modifications are NOT RECOMMENDED), the package can be installed from source through cloning the repository and installing the Python Package in _experimental_ mode.

```
git clone "https://github.com/HumanoidAddverb/trakr_sdk"
cd trakr_sdk/python
python3.8 -m pip install -e .
```
The `-e` flag installs the directory as an experimental package. This means that any changes made in this directory will be reflected in any and all programs that _import_ this package.

## Usage

To test whether the package is correctly installed, try importing it in your code.
```
import trakr_sdk as sdk
```

### A. Initializing Data Buffers
The entire communication with the robot consists of three data types (see `docs/DataTypes.md`):
#### 1. `AlliedDataTypes.Plan`
Consists of the continuous high-level and low-level control commands sent from Client to Robot

#### 2. `AlliedDataTypes.State`
Consists of the continuous feedback from Robot to Client. (State Estimation, Joint States, etc.)

#### 3. `QuadDataTypes.CONFIG_SET`
This holds the configuration of robot defining the type of tasks and safeties to be performed, and is used in two-ways,
- Sent by Client to Robot to set the tasks to be performed (for eg. Left Hand Shake)
- Sent by Robot to Client to indicate the latest configuration present in the Robot.

These data types can be imported as,
```
from trakr_sdk import AlliedDataTypes, QuadDataTypes

plan = AlliedDataTypes.Plan()
state = AlliedDataTypes.State()
config = QuadDataTypes.CONFIG_SET()
```

### B. Setting-Up Connection 
The entire communications with robot have been wrapped under `sdk.Robot` class. \
We set-up an object with the IP Address and Port of Robot,
```
ROBOT_IP = "192.168.3.50"
ROBOT_PORT = 15251

robot = sdk.Robot(ROBOT_IP, ROBOT_PORT)
```

### C. Starting Communications
To initialize a connection, we will need some data that we can send over the socket to perform handshake.
\
The `plan` and `config` buffers created above, hold the default and safe values for commands and configuration(s) of Robot. So, we intialize a connection with the buffers as,
```
robot.setup(config, plan) # -> returns a bool indicating success or failure
```
This function call is a **blocking** call. This means,
- Robot, before it starts accepting user commands and configurations, needs to perform a "Bring-Up" sequence (perform a stand-up and some necessary computations).
- When the client initializes a connection through `robot.setup()`, the code **blocks** until the Robot confirms that the "Bring-Up" sequence is completed.

The call returns a *bool* value, indicating whether the robot has successfully been initialized, and can *fail* in case of any hardware issues that prevent the robot from start-up (to protect the hardware)

### D. Starting Continuous Data-Stream
Now that our connection has been initialized, we start a continuous `while` loop that continuously perform the data-exchange between client and robot. This happens through a single function call,
```
robot.run()
```
Note that this call does not require any arguments. Calling this function sends the last-set commands to robot, and receives the latest feedback from Robot, storing all the data in internal buffers. To access and write to the buffers, we have different calls as,
```
robot.getData(state) # -> populates the RobotFeedback into "state" buffer
robot.getConfig(config) # -> populates the latest configuration of Robot into "config" buffer

robot.setConfig(config) # -> populates the latest configuration from Client into the Robot.
robot.setData(plan) # -> populates the latest commands from Client into the Robot.
```
Not every configuration sent by client will be a *valid* configuration. This is because what is possible to be done at some point in time, may not be possible at a different instance. To check whether the configuration sent to Robot is *valid*, we can call the `getConfigStatus` method as, 
```
robot.getConfigStatus() # -> returns "int",
# 0 -> No configuration was sent to Robot
# 1 -> Configuration was Accepted
# 2 -> Configuration was Rejected
```

And since we'll only be communicating if the robot is "alive" (communications are up), we can check that through,
```
robot.isAlive() # -> returns bool indicating whether communications/robot is/are still alive.
```

> **Note:** The configuration of Robot ("config") is something that is set *occasionally* rather than *continuously*. \
This means that the configuration should be updated *only when there is a change in configuration*. Since the configuration ("CONFIG_SET") is a vast set of different settings, Robot updates the configuration only when User/Client **explicitely** tells the robot to set a configuration, through the `robot.setConfig(config)` call. \
**Call `robot.setConfig(config)` only when needed**. Continously updating configuration will cause the robot to re-apply an already-applied configuration infinite number of times, which can cause *undefined* behavior.

To set the configuration when required, we use a `bool setConfig` to store and check whether a configuration has to be applied, and the code will look something like,
```
...
set_config = True # somewhere in client's code
...

if(set_config):
    robot.setConfig(config)
    set_config = False
```

### E. Minimal Example Code
Bringing everything together, a basic code to communicate with Trakr, will look something like, \
**See: `examples/python/basic.py`**
```
from time import perf_counter_ns, sleep

import trakr_sdk as sdk
from trakr_sdk import QuadDataTypes, AlliedDataTypes

ROBOT_IP = "192.168.3.50"
ROBOT_PORT = 15251

FREQ = 400
LOOP_TIME_NS = 1e9/FREQ

def main():
    robot = sdk.Robot(ROBOT_IP, ROBOT_PORT)

    plan = AlliedDataTypes.Plan()
    state = AlliedDataTypes.State()
    config = QuadDataTypes.CONFIG_SET()

    if(not robot.setup(config, plan)):
        print("[MAIN] Failed to setup robot")
        return False
    
    robot.getConfig(config)

    config_status : int = 0

    set_config : bool = False

    while(robot.isAlive()):
        time_s = perf_counter_ns()
        if(not robot.run()):
            break

        robot.getData(state)
        robot.getConfig(config)

        # this is feedback for last config set.
        # 0 . no config change was requested
        # 1 . Accepted
        # 2 . Rejected 
        config_status = robot.getConfigStatus()

        # do something here

        if(set_config):
            robot.setConfig(config)
            set_config = False

        robot.setData(plan)

        delta_t = perf_counter_ns() - time_s
        if(delta_t < LOOP_TIME_NS):
            sleep((LOOP_TIME_NS - delta_t)/1e9)

    return 0
main()

```
> **Note**: As a general practice, ensure that these method calls happen in the order shown in this example code. This doesn't mean that this is the *only* correct way, but rather that this is a *tried and tested* way that will always work, out of many possible ways that there may be.

### F. Example Implementation(s)
A detailed and tested implementation for this SDK is given in **`examples/python/keyboard.py`**, that allows controlling all available features of Robot through teleoperation with keyboard.

## Best Practices
The `while` loop, considering that the latest data should reach Robot and should be fetched from Robot, must be continuously running at all times. This means,
- Do not use any blocking method calls inside this while loop. If any computation from client requires such a method call, consider creating a different thread (either for Robot or Client's Blocking Call).
- The while loop should be running at ~400Hz. A slower loop might cause less-optimal behavior.

> Additional point to note is that since the SDK maintains its own buffers to send and receive the packets, the packets are sent-and-received in a *different thread*. This thread is created and started (after `setup()`) inside the SDK itself, and runs at a high frequency.

## Documentation

See further documentation in `docs` directory, for details regarding different modes of operation, how to switch between them, etc.