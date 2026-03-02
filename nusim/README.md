# Nusim
A simulated environment for the Nuturtle robot. This package provides a ground-truth simulation authorized to manage robot poses, arena boundaries, and obstacles. It integrates with RViz2 for 3D visualization and simulates physical phenomena such as wheel slip, sensor noise, and collisions.

## Visualization
The simulation environment includes the **red** ground-truth robot, the arena walls, and cylindrical obstacles. It also supports visualization of a **fake sensor** (detecting relative landmark positions).

![Nusim Visualization](images/nusim_rviz.png)

## Launch Files
* **`nusim.launch.xml`**: The primary entry point for the simulation.
    * Starts the `nusimulator` node.
    * Loads the **red** robot model using `nuturtle_description`.
    * Launches `rviz2` with a pre-configured view.
    * **Arguments**:
        * `config_file`: Path to a `.yaml` file to configure the simulator. (Default: `config/basic_world.yaml`).

## Parameters
The `nusimulator` node is configured via the following parameters:

### Simulation Settings
* **`rate`** (int): The frequency of the simulation loop in Hz. Default: `100`.
* **`draw_only`** (bool): If true, the node only publishes markers for walls and obstacles without updating robot physics.

### Robot Initial Pose
* **`x0`**, **`y0`** (float): Initial coordinates of the robot in `nusim/world`.
* **`theta0`** (float): Initial heading of the robot in radians.

### Noise & Physics (New)
* **`input_noise`** (double): Variance of Gaussian noise added to the commanded wheel velocities.
* **`slip_fraction`** (double): Range of uniform random noise applied to simulate wheel slip (applied to the physics layer).
* **`motor_cmd_per_rad_sec`** (double): Scaling factor to convert motor commands to angular velocity.
* **`encoder_ticks_per_rad`** (double): Ticks per radian for simulated encoders.

### Collision & Sensor (New)
* **`collision_radius`** (double): The radius used for robot-obstacle and robot-wall collision detection.
* **`max_range`** (double): Maximum detection range for the fake landmark sensor.
* **`basic_sensor_variance`** (double): Variance of Gaussian noise added to the relative $(x, y)$ landmark measurements.

### Arena & Obstacles
* **`arena_x_length`**, **`arena_y_length`** (float): Dimensions of the arena.
* **`obstacles.x`**, **`obstacles.y`** (double_array): Coordinates of cylindrical obstacles.
* **`obstacles.r`** (float): Radius of the obstacles.

## Topics
* **`~/timestep`** (`std_msgs/msg/UInt64`): Current simulation cycle count.
* **`~/real_walls`** (`visualization_msgs/msg/MarkerArray`): Markers for arena boundaries.
* **`~/real_obstacles`** (`visualization_msgs/msg/MarkerArray`): Ground truth obstacle markers.
* **`red/sensor_data`** (`nuturtlebot_msgs/msg/SensorData`): Simulated wheel encoder readings.
* **`red/joint_states`** (`sensor_msgs/msg/JointState`): Joint positions of the simulated robot.
* **`/fake_sensor`** (`visualization_msgs/msg/MarkerArray`): Noisy relative landmark positions for SLAM.

## Services
* **`~/reset`** (`std_srvs/srv/Empty`): Restores the simulation to the initial state (resets timestep, pose, and clears paths).