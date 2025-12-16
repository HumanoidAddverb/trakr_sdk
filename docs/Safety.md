# trakr_sdk: Safety
Trakr, to protect itself (hardware) from damage, implements three different safeties, and failing *any* one of them causes the robot to *shutdown*. The option to enable/disable the safeties along with the conditions that trigger them can *only* be set through this SDK, and are *reset* upon restarting the robot.
## Explaination

Trakr has three different safeties, and all are running together to protect the hardware. In case the user wants to change/disable the safeties, the same can be done through appropriately sending the configuration to robot. \
Each safety's config has a common attribute - `bool enabled` - which can be set to `false` to disable it.

#### 1. `llsafety` : Per-Joint Low-Level Safety
In this, the safety implements the joint-position limits, joint-velocity limits, and joint-torque limits. If any limit is exceeded, the safety is failed, and robot performs shutdown. \
\
These limits for each joint can be found in `safety_constants.h` or `safety_configs.py`

#### 2. `llheartbeat` : Low-Level Hardware Hearbeat
This safety ensures that the sensor-streams within the robot are supplying fresh data, maintaining a *heartbeat* of the source. If any of the source starts sending *repeated* data over *long* durations, this safety fails, and the robot performs shut-down.\
The heartbeat is maintained over two broad sources,
 1. **IMU**  : In case the connection to IMU is lost (timeout or loose-connection), the robot simply performs the shutdown.
 2. **Per-Joint CAN** : A heartbeat is maintained over *each individual joint*, and if the heartbeat is lost (due to a loose connection or timeout), **_zero_** command is sent to the actuator for which the heartbeat was lost. This means, if the actuator reports back to Robot, the actuator receives a **_zero_** command, and the joint inhibits **_free_** motion. This is done to protect the hardware and prevent any acutator from going *rogue* where it implements *random*/*garbage* commands.

#### 3. `torso` : High-Level Torso Safety
This safety performs a higher-level of monitoring, where it checks whether the robot is still in ideal operating conditions, mainly performing *roll*, *pitch*, and *gyro* limits. This means that the toppling-condition of robot is checked and detected within this safety, upon which this safety fails and the robot performs shutdown (*recovery* : *roll-over*).

## Configuration
- `SafetyDataTypes::CONFIG_SET` : Safety-related robot configurations (see `safety_config.h`)
    - `llsafety` : Per-joint low-level safety (jpos limits, jvel limits, etc.)
    - `llheartbeat` : IMU and per-joint heartbeat safety.
    - `torso` : Higher-level torso orientation safety (roll, pitch, gyro)