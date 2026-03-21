# ME495 Sensing, Navigation and Machine Learning For Robotics
* Chenwan HALLEY Zhong
* Winter 2026
## Package List
This repository consists of several ROS packages:

- **turtlelib** - A standalone C++ library for 2D rigid body transformations ($SE(2)$), differential drive kinematics, geometric primitives, SVG visualization, and the core **Extended Kalman Filter (EKF)** math.
- **nuslam** - The SLAM package implementing a **Feature-Based EKF SLAM** algorithm
  with both known and **unknown data association** (Euclidean distance gating).
  Achieves < 3.5 cm landmark estimation error and < 3 mm robot pose error after
  one closed loop in simulation.
- **nusim** - A simulation package providing a ground-truth environment with arena walls, obstacles, physical noise modeling (slip/input noise), and a kinematic robot simulator.
- **nuturtle_control** - The core control package that implements odometry, motor command translation, and circular trajectory nodes.
- **nuturtle_description** - URDF files, meshes, and launch files for visualizing the Nuturtle Burger robot in different configurations (Red, Blue, Green).
## System Demonstrations

### Simulation Testing
This video demonstrates the overlay of the simulated "ground-truth" robot (red) and the odometry-estimated robot (blue). Under ideal simulation conditions, the two robots remain perfectly synchronized.

https://private-user-images.githubusercontent.com/189086001/546817157-e214b7b2-40f2-4e38-9069-24b56f624eb2.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzA2NTc4MzgsIm5iZiI6MTc3MDY1NzUzOCwicGF0aCI6Ii8xODkwODYwMDEvNTQ2ODE3MTU3LWUyMTRiN2IyLTQwZjItNGUzOC05MDY5LTI0YjU2ZjYyNGViMi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMjA5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDIwOVQxNzE4NThaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT03Y2I5M2VjZGFlYzMzY2EyMWQ3NDYwNjQ0MDY4OTI5YjliZjMwNmU5YTgwYzI4NTgyNjQ5MzQ4MDA5OWI4NDFmJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.5U_5JUc6EiTUyt_kqoAN5vxM8zumbWMPl7Ds3VXOKog

### Physical Robot Testing
This video shows the TurtleBot (Leonardo) performing circular maneuvers in the lab. The robot is driven in a circle and returns to its initial configuration to measure odometry drift.

https://private-user-images.githubusercontent.com/189086001/547237008-a0e7902e-7302-405b-b283-fa60a3cd83ea.MOV?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzA2NTc4NjgsIm5iZiI6MTc3MDY1NzU2OCwicGF0aCI6Ii8xODkwODYwMDEvNTQ3MjM3MDA4LWEwZTc5MDJlLTczMDItNDA1Yi1iMjgzLWZhNjBhM2NkODNlYS5NT1Y_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMjA5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDIwOVQxNzE5MjhaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT02MTBhN2M3MjQ2YzNiMzJlMzFhMTYzZDk0ZWU4NTZjMWY2YThkZTQ1ODQzNGI5MTRlZWU1OTM2NTc5NDg4ZTJlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.I-PXh0Ou9jc2c9hPaqpDVlqsvGGblEz_6A8i0gfCvHw

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