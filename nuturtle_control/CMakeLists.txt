cmake_minimum_required(VERSION 3.22)
project(nuturtle_control)

# Enable C++23 as per turtlelib standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Apply mandatory warning flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Generate Custom Interfaces ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InitialPose.srv"
  DEPENDENCIES geometry_msgs
)

# --- Executables ---

# 1. Turtle Control Node
add_executable(turtle_control src/turtle_control.cpp)
target_link_libraries(turtle_control
  PRIVATE
  rclcpp::rclcpp
  turtlelib::turtlelib
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nuturtlebot_msgs::nuturtlebot_msgs__rosidl_typesupport_cpp
)

# 2. Odometry Node
add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry 
  PRIVATE
  rclcpp::rclcpp 
  turtlelib::turtlelib
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
)
# Ensure odometry links to the generated InitialPose service
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(odometry PRIVATE "${cpp_typesupport_target}")

# --- Installation ---
install(TARGETS
  turtle_control
  odometry
  DESTINATION lib/${PROJECT_NAME}
)

# --- Testing and Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()