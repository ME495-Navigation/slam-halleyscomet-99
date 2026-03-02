cmake_minimum_required(VERSION 3.22)
project(nuslam)

# --- Compiler Settings ---
# Enable C++23 to match turtlelib and modern standards
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlelib REQUIRED) # Your custom math library
find_package(nuturtlebot_msgs REQUIRED)

# --- Executable Targets ---

# Define the SLAM node
add_executable(slam src/slam.cpp)

# Link dependencies
target_link_libraries(slam PUBLIC
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${nuturtlebot_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${visualization_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
      tf2::tf2
      tf2_ros::static_transform_broadcaster_node
      tf2_ros::tf2_ros
      turtlelib::turtlelib
    )

# --- Installation ---

# Install the executables to the lib/nuslam directory
install(TARGETS
  slam
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files (URDFs, YAMLs, RViz configs)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing and Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Ament Package Macro ---
ament_package()