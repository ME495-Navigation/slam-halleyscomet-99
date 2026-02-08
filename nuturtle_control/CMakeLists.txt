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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Interfaces ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InitialPose.srv"
  "srv/Control.srv"
  DEPENDENCIES geometry_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- Nodes ---

# 1. Turtle Control
add_executable(turtle_control src/turtle_control.cpp)
target_link_libraries(turtle_control
  PRIVATE
  rclcpp::rclcpp
  turtlelib::turtlelib
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nuturtlebot_msgs::nuturtlebot_msgs__rosidl_typesupport_cpp
)

# 2. Odometry
add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry
  PRIVATE
  rclcpp::rclcpp
  turtlelib::turtlelib
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  "${cpp_typesupport_target}"
)

# 3. Circle
add_executable(circle src/circle.cpp)
target_link_libraries(circle
  PRIVATE
  rclcpp::rclcpp
  turtlelib::turtlelib
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  "${cpp_typesupport_target}"
)

# --- Installation ---
install(TARGETS turtle_control odometry circle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv launch config DESTINATION share/${PROJECT_NAME})

install(FILES
  tests/turtle_control_test.launch.xml
  tests/turtle_odom_test.launch.xml
  tests/turtle_circle_test.launch.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(catch_ros2 REQUIRED)

  # Control Test
  add_executable(turtle_control_test tests/turtle_control_test.cpp)
  target_link_libraries(turtle_control_test
    PRIVATE
    catch_ros2::catch_ros2_with_node_main
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    nuturtlebot_msgs::nuturtlebot_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  )

  # Odom Test
  add_executable(turtle_odom_test tests/turtle_odom_test.cpp)
  target_link_libraries(turtle_odom_test
    PRIVATE
    catch_ros2::catch_ros2_with_node_main
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    "${cpp_typesupport_target}"
  )

  # Circle Test
  add_executable(turtle_circle_test tests/turtle_circle_test.cpp)
  target_link_libraries(turtle_circle_test
    PRIVATE
    catch_ros2::catch_ros2_with_node_main
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    "${cpp_typesupport_target}"
  )

  install(TARGETS turtle_control_test turtle_odom_test turtle_circle_test
    DESTINATION lib/${PROJECT_NAME})

  catch_ros2_add_integration_test(turtle_control_test
    LAUNCH_FILE turtle_control_test.launch.xml
  )
  catch_ros2_add_integration_test(turtle_odom_test
    LAUNCH_FILE turtle_odom_test.launch.xml
  )
  catch_ros2_add_integration_test(turtle_circle_test
    LAUNCH_FILE turtle_circle_test.launch.xml
  )
endif()

ament_package()