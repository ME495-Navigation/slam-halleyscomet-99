# ME495 Sensing, Navigation and Machine Learning For Robotics
* Chenwan HALLEY Zhong
* Winter 2026
## Package List
This repository consists of several ROS packages:

- **turtlelib** - A standalone C++ library for 2D rigid body transformations ($SE(2)$), geometric primitives, and SVG visualization.
- **nuturtle_description** - A package containing the URDF files, meshes, and launch files for visualizing the Nuturtle Burger robot in RViz.
- **nusim** - A simulation package that provides a ground-truth environment, including arena walls, cylindrical obstacles, and robot state tracking.

## Build and Usage
To build all packages in this repository, navigate to the root of your workspace and run:

```bash
colcon build
source install/setup.bash