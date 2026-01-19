cmake_minimum_required(VERSION 3.20)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(turtlelib REQUIRED)

add_executable(nusimulator src/nusim.cpp)
target_link_libraries(nusimulator PUBLIC
      ${geometry_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
      rclcpp::rclcpp
      tf2_ros::static_transform_broadcaster_node
      tf2_ros::tf2_ros
)

install(TARGETS
  nusimulator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_lint_auto_find_test_dependencies REDUNDANT_CHECK ON)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
