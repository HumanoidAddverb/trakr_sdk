# trakr_sdk: DataTypes
> Data types in C++ are similar to same as in Python. Namespaces in C++ have been implemented as multi-file imports in Python. So, for a XX::XX data type in C++, look for XX.XX in Python.

The entire communication with the robot consists of three data types, which themselves are collection of smaller data types as,
## 1. `AlliedDataTypes::Plan`(C++) or `AlliedDataTypes.Plan`(Python)
Consists of the continuous high-level and low-level control commands sent from Client to Robot. \
Has two attributes (NDOF = 12),
- `torso` : `TorsoState`
    - `pos` : 6x1 : (roll, pitch, yaw, x, y, z) *(rad, m)*
    - `vel` : 6x1 : (3 ang vel,  3 lin vel) *(rad/s, m/s)*

- `joint` : `JointState`
    - `pos` : NDOF x 1 : Joint Position (rad)
    - `vel` : NDOF x 1 : Joint Velocity (rad/s)
    - `kp` : NDOF x 1 : Joint Stiffness (Nm/rad)
    - `kd` : NDOF x 1 : Joint Damping (Nm.s/rad)
    - `tor` : NDOF x 1 : Feed-Forward Joint Torques (Nm)

The order of *NDOF = 12* joints for this frame is as follows,
- Front-Left 
    - Adduction : 0
    - Hip : 1
    - Knee : 2
- Front-Right
    - Adduction : 3
    - Hip : 4
    - Knee : 5
- Rear-Left
    - Adduction : 6
    - Hip : 7
    - Knee : 8
- Rear-Right
    - Adduction : 9
    - Hip : 10
    - Knee : 11

## 2. `AlliedDataTypes::State`(C++) or `AlliedDataTypes.State`(Python)
Consists of the continuous feedback from Robot to Client. Has four attributes,
- `torso` : `TorsoState`
    - `pos` : 6x1 : Position Estimate (m)
    - `vel` : 6x1 : Velocity Estimate (m/s)
- `joint` : `JointState`
    - `pos` : NDOF x 1 : Joint Position Feedback (rad)
    - `vel` : NDOF x 1 : Joint Velocity Feedback (rad/s)
    - `tor` : NDOF x 1 : Joint Torque Feedback (Nm)
    - `kp` : NDOF x 1 : *Unused*
    - `kd` : NDOF x 1 : *Unused*
- `imu` : `IMUState`
    - `acc` : 3 x 1 : Linear Acceleration (Calibrated) (m/s2)
    - `gyro` : 3 x 1 : Angular Velocity (Calibrated) (rad/s)
    - `mag` : 3 x 1 : *Unused*
    - `euler` : 3 x 1 : (roll, pitch, yaw) (rad)
- `power` : `BatteryState` : *Unused*

## 3. `QuadDataTypes::CONFIG_SET`(C++) or `QuadDataTypes.CONFIG_SET`(Python)
This holds the configuration of robot defining the type of tasks and safeties to be performed, and is used in two-ways,
- Sent by Client to Robot to set the tasks to be performed (for eg. Left Hand Shake)
- Sent by Robot to Client to indicate the latest configuration present in the Robot.

It is a collection of smaller configuration sets, and broadly has three attributes,
- `master` : `ROBOT_CONFIG` : Global robot configurations
    - `shutdown` : `bool` : Whether to perform Shutdown (Sit-Down Sequence, will not stand up again)
    - `exit` : `bool` : Whether to shutdown and stop the on-board program (disables actuators).
    - `killed` : `bool` : Whether to emergency-kill the power to actuators (CAN DAMAGE HARDWARE).
- `motion` : `MotionDataTypes::CONFIG_SET` : Motion-related robot configurations
    - See `Modes.md`
- `safety` : `SafetyDataTypes::CONFIG_SET` : Safety-related robot configurations (see `safety_config.h`)
    - See `Safety.md`