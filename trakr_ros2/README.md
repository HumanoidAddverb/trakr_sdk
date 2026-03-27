# trakr_ros2 Package
A minimal package that allows the robot to be controlled over a ROS2 network.\
Builds on top of the same socket/sdk (trakr_cpp_sdk), it streams the state of robot over topics, with the different configurations of robot available as different services that can be triggered.

The package has the same OS/Architecture requirements as the *trakr_cpp_sdk* package.

## Dependencies
It assumes that a ROS2 installation is present on the system.\

Uses the following packages (shipped with the base ROS2 installation),
- std_msgs
- geometry_msgs
- sensor_msgs
- std_srvs

## Build
It is a ROS2 package, that can be built in a ROS2 workspace with `colcon build`, or can be directly built as a CMake package after sourcing the ROS setup script.

## Usage
The package creates a single node `trakr_ros2`, responsible for communicating with the robot as well as with the ROS2 network.

### 1. Publishers
The node publishes the state of robot over different topics as,
- `/trakr/joint_states` : `sensor_msgs/JointState`, containing the joint data (pos, vel, tor), joint order and timestamp
- `/trakr/imu` : `sensor_msgs/Imu`, containing the accelerometer and gyroscope data
- `/trakr/body_twist` : `geometry_msgs/Twist`, containing the estimates of linear and angular velocities given by the state estimator running on the robot.

### 2. Subscriptions
The node subscribes the command for the robot as,
- `/trakr/cmd_vel` : `geometry_msgs/Twist`, for specifically 3DOF control - lin_vel_x, lin_vel_y and ang_vel_z.

### 3. Services
The several configurations in which the robot can operate in (for eg. AI Mode Walk or Classical Mode), have been interfaced as services with the standard `Trigger` definition. They do not require any payload in request, and are **blocking** until the configuration change has been completed by the robot.
- `/trakr/ai_mode/walk` : `std_srvs/Trigger`, for switching to **AIMode-Walk**
- `/trakr/ai_mode/climb` : `std_srvs/Trigger`, for switching to **AIMode-Climb**, allowing the robot to climb stairs/discrete-steps.
- `/trakr/classical_mode/trot` : `std_srvs/Trigger`, for switching to **Classical Mode Trotting**
- `/trakr/gesture_mode/orientation` :  : `std_srvs/Trigger`, for switching to **Orientation-Mode in Gesture Mode**
- `/trakr/gesture_mode/standup` : `std_srvs/Trigger`, for making the robot go to **stand-up** joint configuration
- `/trakr/gesture_mode/sitdown` : `std_srvs/Trigger`, for making the robot go to **sit-down** joint configuration
- `/trakr/gesture_mode/left_handshake` : `std_srvs/Trigger`, for making the robot perform **left hand shake**
- `/trakr/gesture_mode/right_handshake` : `std_srvs/Trigger`, for making the robot perform **right hand shake**

## Additional Notes
The `trakr_ros2` node is directly linked with the robot over socket. This means that closing the node (due to network shutdown or otherwise), will make the robot perform exit sequence.

If the robot fails (due to any safety), the robot will perform sit-down **but will remain alive** - waiting for the ROS node to shutdown, before it exits its own code.

This effectively means that the lifetime of both, the node and the robot, are linked together, meaning that both will need to be relaunched in-case of any failure in safety or otherwise.

## Documentation

See further documentation in `docs` directory, for details regarding different modes of operation, etc.