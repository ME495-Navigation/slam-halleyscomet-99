# ME495 Sensing, Navigation and Machine Learning For Robotics
* Chenwan HALLEY Zhong
* Winter 2026
## Package List
This repository consists of several ROS packages:

- **turtlelib** - A standalone C++ library for 2D rigid body transformations ($SE(2)$), differential drive kinematics, geometric primitives, and SVG visualization.
- **nuturtle_description** - URDF files, meshes, and launch files for visualizing the Nuturtle Burger robot.
- **nusim** - A simulation package providing a ground-truth environment with arena walls, obstacles, and a kinematic robot simulator.
- **nuturtle_control** - The core control package that implements odometry, motor command translation, and circular trajectory nodes.

## System Demonstrations

### Simulation Testing
This video demonstrates the overlay of the simulated "ground-truth" robot (red) and the odometry-estimated robot (blue). Under ideal simulation conditions, the two robots remain perfectly synchronized.

[Watch Simulation Video](URL_TO_YOUR_SIM_VIDEO_HERE)

### Physical Robot Testing
This video shows the TurtleBot (Leonardo) performing circular maneuvers in the lab. The robot is driven in a circle and returns to its initial configuration to measure odometry drift.

[Watch Physical Robot Video](URL_TO_YOUR_REAL_ROBOT_VIDEO_HERE)

## Build Instructions
To build all packages in this repository, navigate to the root of your ROS 2 workspace and run:

```bash
colcon build
source install/setup.bash
```

## Quick Start
To launch the full simulation environment with RViz visualization:
```bash
ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim cmd_src:=teleop
```

To run on the physical Turtlebot (Localhost):
```bash
ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=circle
```

## Physical Testing Results

Following the **Task F.3** experiment (driving the robot in a full circle and returning to the starting marker), the final odometry position and calculated drift were recorded as follows:

| Coordinate | Final Value (Odometry) |
| :----- | :--- |
| **$x$** | $-0.07397$ m |
| **$y$** | $0.09907$ m |
| **$\theta$** | $0.78177$ rad |

**Total Euclidean Error:** $0.1236$ meters



Detailed analysis and technical documentation for each specific node can be found in the `README.md` files within the individual packages.