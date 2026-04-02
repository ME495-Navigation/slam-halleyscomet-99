# ME495 Sensing, Navigation and Machine Learning For Robotics
* Chenwan HALLEY Zhong
* Winter 2026

## Package List

This repository consists of several ROS packages:

- **turtlelib** - A standalone C++ library for 2D rigid body transformations (SE(2)), differential drive kinematics, geometric primitives, SVG visualization, and Extended Kalman Filter math.
- **nuslam** - A Feature-Based EKF SLAM package with unknown data association using Euclidean distance gating. Achieves < 3.5 cm landmark estimation error and < 3 mm robot pose error after one closed loop in simulation.
- **nusim** - A simulation package providing a ground-truth environment with arena walls, cylindrical obstacles, physical noise modeling (slip and input noise), and a kinematic robot simulator.
- **nuturtle_control** - The core control package implementing odometry, motor command translation, and circular trajectory nodes.
- **nuturtle_description** - URDF files, meshes, and launch files for visualizing the Nuturtle Burger robot in red, blue, and green configurations.

## Build Instructions
```bash
colcon build
source install/setup.bash
```

## System Demonstrations

### Physical Robot: EKF SLAM with Landmark Detection

This video (×4.0) shows the TurtleBot navigating among cylindrical obstacles in the lab while the EKF SLAM algorithm estimates the robot pose and landmark positions in real time. The green robot and path represent the SLAM estimate; the blue robot and path show raw odometry drift.

[https://private-user-images.githubusercontent.com/189086001/567344568-8bb6b587-bf44-4d39-a059-0b445686b260.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzQxMzQ4NjEsIm5iZiI6MTc3NDEzNDU2MSwicGF0aCI6Ii8xODkwODYwMDEvNTY3MzQ0NTY4LThiYjZiNTg3LWJmNDQtNGQzOS1hMDU5LTBiNDQ1Njg2YjI2MC5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMzIxJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDMyMVQyMzA5MjFaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT0yZjc2NTliYWY2ZTUyMGQ5Njc0YzIyMWVhYTJmMzA2ZGRmMjY4MjgyZTc2ZDE5NDM4N2MxNDcwYTQzNjQyNmFmJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.grxmoYAwC2hde4gwg4mw5aqbH1wFaxoKN3bePoGix_A](https://private-user-images.githubusercontent.com/189086001/573188305-c89a96eb-0b5c-4232-9dfe-cec43ad495bf.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzUxNDkwNzUsIm5iZiI6MTc3NTE0ODc3NSwicGF0aCI6Ii8xODkwODYwMDEvNTczMTg4MzA1LWM4OWE5NmViLTBiNWMtNDIzMi05ZGZlLWNlYzQzYWQ0OTViZi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNDAyJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDQwMlQxNjUyNTVaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT0zNjhhNzA4MDM4Mzc3MmNiMTFmYjU0NjcyN2UzMDY3NzhlNWMzNzU3ZDA1MzJjYmJmMTczY2VlZDI2NzFiZWViJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.rVUn6eg5FvJE2jwF_5BpMjZVAuW3xiwbqv9d9WtBnck)

### Simulation: EKF SLAM with Unknown Data Association

This screencast (x5.0) shows the full EKF SLAM pipeline running in simulation. The green robot (SLAM estimate) closely tracks the red robot (ground truth) while the blue robot (odometry) drifts over time. Green cylinders converge to the true obstacle positions after a few laps.

https://private-user-images.githubusercontent.com/189086001/567272783-8415716e-14bf-44ae-9f44-1bc245433fd7.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzQxMzQ5NTksIm5iZiI6MTc3NDEzNDY1OSwicGF0aCI6Ii8xODkwODYwMDEvNTY3MjcyNzgzLTg0MTU3MTZlLTE0YmYtNDRhZS05ZjQ0LTFiYzI0NTQzM2ZkNy5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMzIxJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDMyMVQyMzEwNTlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT04ZDQ5ODlkNjA0ODI5YTBjYzQ3OGViMDA1MTgyOGE1NGE0NmE1YTA4MWFhYWUyOGJjNTI4OGI3YTQ4MzFkZWQxJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.SH_Cdo0WUYJrBG3A3kRreEg3UJFEkSS78E8NnERraXU

### Physical Robot: Odometry Drift Measurement

This video shows the TurtleBot performing circular maneuvers in the lab. The robot is driven in a circle and returns to its initial configuration to measure odometry drift.

https://private-user-images.githubusercontent.com/189086001/567356370-85e9fb6a-64de-495a-b21b-be067f3163d3.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzQxMzk2ODksIm5iZiI6MTc3NDEzOTM4OSwicGF0aCI6Ii8xODkwODYwMDEvNTY3MzU2MzcwLTg1ZTlmYjZhLTY0ZGUtNDk1YS1iMjFiLWJlMDY3ZjMxNjNkMy5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMzIyJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDMyMlQwMDI5NDlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1lNjU0YjE2NjgxNDg4N2JkOGVhYzY0YzVlOTJhOWU5YjI4OTI1ZjRjMjcwNjU5MmQ4OWZmYzY4MDE4MjM3M2ZhJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.e_NqXk2M-u5k_b87GKg0NzAbn2qkcXPkoM-ADgrgbD4

### Simulation: Odometry Overlay

This video shows the simulated ground-truth robot (red) and the odometry-estimated robot (blue) under ideal simulation conditions. The two robots remain synchronized when no slip or input noise is applied.

https://private-user-images.githubusercontent.com/189086001/546817157-e214b7b2-40f2-4e38-9069-24b56f624eb2.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzA2NTc4MzgsIm5iZiI6MTc3MDY1NzUzOCwicGF0aCI6Ii8xODkwODYwMDEvNTQ2ODE3MTU3LWUyMTRiN2IyLTQwZjItNGUzOC05MDY5LTI0YjU2ZjYyNGViMi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMjA5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDIwOVQxNzE4NThaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT03Y2I5M2VjZGFlYzMzY2EyMWQ3NDYwNjQ0MDY4OTI5YjliZjMwNmU5YTgwYzI4NTgyNjQ5MzQ4MDA5OWI4NDFmJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.5U_5JUc6EiTUyt_kqoAN5vxM8zumbWMPl7Ds3VXOKog

## Quick Start

Simulation with teleop:
```bash
ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim cmd_src:=teleop
```

EKF SLAM in simulation:
```bash
ros2 launch nuslam unknown_data_assoc.launch.xml cmd_src:=circle
```

EKF SLAM on the physical TurtleBot — on the robot:
```bash
ros2 launch nuslam turtlebot_bringup.launch.xml
```

On your computer:
```bash
ros2 launch nuslam pc_bringup.launch.xml cmd_src:=teleop
```

Physical robot (odometry only):
```bash
ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=circle
```

## Physical Testing Results

### Odometry Drift (circle test)

Following the odometry drift experiment, the robot was driven in a full circle and returned to the starting marker. The final odometry position was:

| Coordinate | Final Value |
|---|---|
| x | −0.07397 m |
| y |  0.09907 m |
| θ |  0.78177 rad |

**Total Euclidean Error:** 0.1236 m

### EKF SLAM (real robot)

The robot was driven among five cylindrical obstacles. Final pose after returning near the starting position:

| Estimate | x [m] | y [m] | θ [rad] | Total error [m] |
|---|---|---|---|---|
| Odometry vs Ground Truth | 0.000 | 0.000 | 0.000 | 0.000 |
| SLAM vs Ground Truth     | 0.071 | −0.093 | 3.030 | 0.117 |

The odometry error is near zero because the robot was driven a short distance, insufficient to accumulate measurable encoder drift. Over longer paths the SLAM correction becomes essential. Real-world performance is limited by the ~2.5 Hz LiDAR update rate and higher physical sensor noise compared with the simulation model.

## Detailed Documentation

Package-level documentation, parameter descriptions, and topic listings can be found in the `README.md` files within each individual package directory.
