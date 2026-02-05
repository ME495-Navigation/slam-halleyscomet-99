cmake_minimum_required(VERSION 3.22)
project(nuturtle_control)

# Enable C++23
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
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtle_description REQUIRED)

# --- Executable ---
add_executable(turtle_control src/turtle_control.cpp)

target_link_libraries(turtle_control
  PRIVATE
  rclcpp::rclcpp
  turtlelib::turtlelib
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nuturtlebot_msgs::nuturtlebot_msgs__rosidl_typesupport_cpp
)

# --- Installation ---
install(TARGETS
  turtle_control
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