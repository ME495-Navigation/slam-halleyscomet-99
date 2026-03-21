cmake_minimum_required(VERSION 3.22)
project(nuslam)

# --- Compiler Settings ---
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
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Executable Targets ---

# 1. SLAM Node
add_executable(slam src/slam.cpp)
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
      Eigen3::Eigen
)

# 2. Landmarks Node
add_executable(landmarks src/landmarks.cpp)
target_link_libraries(landmarks PUBLIC
      ${sensor_msgs_TARGETS}
      ${visualization_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
      turtlelib::turtlelib
      Eigen3::Eigen
)

# --- Installation ---

install(TARGETS
  slam
  landmarks
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing and Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(circle_tests tests/test_circle.cpp)
  target_link_libraries(circle_tests turtlelib::turtlelib)
endif()

ament_package()