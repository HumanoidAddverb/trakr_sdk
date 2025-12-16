# trakr_sdk: Modes
Trakr can be operated in different modes allowing different behavior of robot for same commands. These modes can be selected on different buttons on wireless controller, keyboard, or through setting configuration over socket. \
While explaining the differences in these modes, this guide focuses mainly on setting configuration over socket. For wireless controller and keyboard, refer to User Manual for control bindings.

## Explaination


Trakr has five different modes of operation/motion, out of which **four** can be chosen as required, while one is performed *automatically* by robot as needed.
#### 1. Gesture Mode
This mode allows the robot to perform some pre-defined trajectories to perform required actions. As of now, following gestures have been implemented,
- StandUp
- SitDown
- Orientation
- LeftHandShake
- RightHandShake

(More gestures will be added in the future).
> Robot performs these gestures *in-place* without locomoting, hence, from software-point of implementation, this mode has been differentiated as *eGesture*, while the remaining modes are implemented under the umbrella of *eMotion*.

#### 2. Classical Mode
Robot performs locomotion with predefined *trotting* gait, along with online disturbance rejection. It accurately tracks the velocity commands and is ideal mode to perform point-to-point locomotion (client has to implement the high-level controller, but the position-estimation is accurate). This mode has been named to due to the fact that this mode implements *Model-Predictive Control*, now referred as *Classical* control with the shift of focus towards AI. \
\
The robot in this mode,
- Can walk only on flat-terrains with discrete steps upto ~5cm
- Cannot walk in slippery/mud/uneven terrain.
- Reaches accurate speeds upto 1m/s.
- Walks with a defined *Trotting* gait.

#### 3. AI Mode
Robot performs locomotion without any predefined gait, along with comparatively high disturbance rejection. It reaches speeds higher than Classical Mode, and is ideal for longer-durations of operation, but generally over-shoots the veloctiy command given. This mode has been named so due to the fact that this mode deploys a RL policy trained in simulation, broadly referred to as AI. \
\
The robot in this mode,
- Can walk on slippery/uneven/muddy terrains.
- Cannot walk on discrete steps.
- Reaches speeds upto ~1.5m/s to ~2m/s
- Walks without a pre-defined gait.

#### 4. Developer Mode
This mode allows user to perform low-level control of robot, where the joint-commands sent by user over socket are directly sent to actuators. To better help with development and safety, robot implements **joint-level damping** when user-commands to all joints is zero. \
This is decided on the basis of the `kp`, `kd`, and `tor` commands. If all the three are *zero* for all the joints (received from user over socket), the robot performs joint-level damping with `kd` = `1`, and otherwise sends the user-commands to actuators.

#### 5. Recovery Mode (Automatic)
Upon failing any safety implemented by the robot, the robot goes in *Recovery Mode* where it stops accepting user-commands and implements pre-defined trajectories to safely shut-down the robot. \
This behavior is broadly of two types:
- **Sit-Down** : If the robot is oriented correctly when a safety is violated, the robot directly performs sit-down and waits for user to give *"exit"* command, post which powers to all actuators are killed.
- **Roll-Over** : If the robot has toppled over after a safety is violated, the robot performs a roll-over sequence, depending on the configuration of robot, and tries to roll-over itself to bring itself back to the sitting position. It tries this manuveur *once*, and does not try again even if it ends up rolled-over again. In case this sequence should be aborted (if the robot is stuck), commanding the *"shutdown"* command will immediately stop this manuveur and robot will go to sitting position.


## Configuration
- `MotionDataTypes::CONFIG_SET` : Motion-related robot configurations
    - `planner` : `enum TaskTypes` : Deciding whether to perform `TaskTypes::eMotion` or `TaskTypes::eGesture`
    - `sequence` : Configuration for Gesture to be performed (used if `planner` == `TaskTypes::eGesture`)
        - `type` : `enum GestureTypes` : Type of gesture to be performed
            - `GestureTypes::eNone`
            - `GestureTypes::eStandUp`
            - `GestureTypes::eSitDown`
            - `GestureTypes::eOrientation`
            - `GestureTypes::eLeftHandShake`
            - `GestureTypes::eRightHandShake`
    - `strategy` : Configuration for motion mode to be performed
        - `type` : `enum MotionModes` : Type of motion to be performed
            - `MotionModes::eNone`
            - `MotionModes::eClassicalMode`
            - `MotionModes::eAIMode`
            - `MotionModes::eDeveloperMode`
        - `seq` : Type of Gait to be performed during motion (Only used for mainly `eClassicalMode`)
            - `MotionGaits::eStance`
            - `MotionGaits::eTrot`