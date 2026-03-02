# Nuturtle Control
This package implements the control system and odometry tracking for the Nuturtle robot. It bridges the gap between high-level geometry commands and the physical/simulated hardware, while maintaining a real-time estimate of the robot's pose.

## Visualization
Below is a demonstration of the `start_robot.launch.xml` in simulation mode. 
- **Red Robot**: Ground truth from `nusimulator`.
- **Blue Robot**: Pure odometry pose estimated by the `odometry` node.
- **Green Robot** (New): Estimated pose from the SLAM algorithm (if running `nuslam`).



## Launch Files
* **`start_robot.launch.xml`**: The primary entry point for controlling either a simulated or a real robot.
    * **Arguments**:
        * `cmd_src` (string): The source of movement commands. Options: `circle`, `teleop`, or `none`. (Default: `circle`).
        * `robot` (string): The robot platform. Options: `nusim` (simulation), `localhost` (real Turtlebot), or `none`. (Default: `nusim`).
        * `use_rviz` (bool): Whether to launch RViz2 for visualization. (Default: `true`).

## Nodes
### `turtle_control`
Translates between standard ROS messages and Nuturtle-specific hardware messages.
* **Subscribes**: 
    * `cmd_vel` (`geometry_msgs/msg/Twist`): Target velocity.
    * `sensor_data` (`nuturtlebot_msgs/msg/SensorData`): Encoder feedback from the robot.
* **Publishes**:
    * `wheel_cmd` (`nuturtlebot_msgs/msg/WheelCommands`): Commands sent to the motors (scaled to -265 to 265).
    * `joint_states` (`sensor_msgs/msg/JointState`): Current wheel positions in radians.

### `odometry`
Tracks the robot's pose over time based on wheel joint states.
* **Subscribes**:
    * `joint_states` (`sensor_msgs/msg/JointState`): Radian wheel positions.
* **Publishes**:
    * `odom` (`nav_msgs/msg/Odometry`): The calculated pose and velocity in the `odom` frame.
    * `~/odom_path` (`nav_msgs/msg/Path`): The uncorrected history of robot positions.
* **Broadcasts**:
    * **TF Transform**: `odom` -> `blue/base_footprint`. 
    * *Note: When running SLAM, an additional transform `nusim/world` -> `blue/base_footprint` may be used to visualize pure drift.*

### `circle`
A high-level controller that drives the robot in a circular path.
* **Publishes**:
    * `cmd_vel` (`geometry_msgs/msg/Twist`): Constant velocity command.
* **Services**:
    * `control` (`nuturtle_control/srv/Control`): Set the `velocity` and `radius` for the circle.

## Usage & Command Lines

### 1. Launching Simulation
To run the simulator with the controller and odometry overlay:

```bash
ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim cmd_src:=circle
```

### 2. Launching Real Robot
To run on the physical Turtlebot (ensure `numsr_turtlebot` and `ROS_DOMAIN_ID` are set):

```bash
ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=teleop
```

### 3. Controlling the Robot

* **Teleop via Keyboard**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* **Driving in a Circle (Service Call)**:
```bash
ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.2, radius: 0.5}"
```

* **Resetting Odometry Pose:**:
```bash
ros2 service call /initial_pose nuturtle_control/srv/InitialPose "{x: 0.0, y: 0.0, theta: 0.0}"
```