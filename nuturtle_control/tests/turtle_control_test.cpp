/// \file
/// \brief Integration tests for the turtle_control node's ROS API.

#include <catch2/catch_test_macros.hpp>
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

// Global variables to store the latest message data for assertions
static int last_left_vel = 0;
static int last_right_vel = 0;
static bool wheel_cmd_received = false;
static bool joint_state_received = false;
static sensor_msgs::msg::JointState last_joint_state;

/// \brief Callback for the wheel_cmd subscription
/// \param msg - The received WheelCommands message
void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  last_left_vel = msg->left_velocity;
  last_right_vel = msg->right_velocity;
  wheel_cmd_received = true;
}

/// \brief Callback for the joint_states subscription
/// \param msg - The received JointState message
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  last_joint_state = *msg;
  joint_state_received = true;
}

TEST_CASE("turtle_control_integration", "[turtle_control]") {
  auto node = rclcpp::Node::make_shared("turtle_control_test_node");

  // Declare and get test_duration parameter
  node->declare_parameter<double>("test_duration", 2.0);
  const auto TEST_DURATION = node->get_parameter("test_duration").as_double();

  // --- Test Case 1: Pure Translation ---
  SECTION("Pure translation results in appropriate wheel_cmd") {
    wheel_cmd_received = false;
    last_left_vel = 0;
    last_right_vel = 0;

    auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, &wheel_cmd_callback);
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Wait for discovery: ensure turtle_control is actually subscribed to cmd_vel
    auto start_discovery = node->get_clock()->now();
    while (pub->get_subscription_count() == 0 &&
      (node->get_clock()->now() - start_discovery) < 2s)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.5;   // Higher velocity to overcome small scaling truncation
    twist.angular.z = 0.0;

    auto start_time = node->get_clock()->now();
    // Continue publishing until a non-zero message is received or timeout occurs
    while (rclcpp::ok() &&
      (node->get_clock()->now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
    {
      pub->publish(twist);
      rclcpp::spin_some(node);

      // Break if received a command that is actually non-zero
      if (wheel_cmd_received && last_left_vel != 0) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }

    REQUIRE(wheel_cmd_received);
    // In pure translation, wheels should spin at the same speed and direction
    CHECK(last_left_vel == last_right_vel);
    CHECK(last_left_vel > 0);
  }

  // --- Test Case 2: Pure Rotation ---
  SECTION("Pure rotation results in appropriate wheel_cmd") {
    wheel_cmd_received = false;
    last_left_vel = 0;
    last_right_vel = 0;

    auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, &wheel_cmd_callback);
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Wait for discovery
    auto start_discovery = node->get_clock()->now();
    while (pub->get_subscription_count() == 0 &&
      (node->get_clock()->now() - start_discovery) < 2s)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 2.0;   // Higher angular velocity

    auto start_time = node->get_clock()->now();
    while (rclcpp::ok() &&
      (node->get_clock()->now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
    {
      pub->publish(twist);
      rclcpp::spin_some(node);

      if (wheel_cmd_received && last_right_vel != 0) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }

    REQUIRE(wheel_cmd_received);
    // In pure rotation, wheels move in opposite directions.
    // For CCW (+z) rotation, right wheel forward (+), left wheel backward (-)
    CHECK(last_left_vel == -last_right_vel);
    CHECK(last_right_vel > 0);
  }

  // --- Test Case 3: Encoder to Joint States ---
  SECTION("Encoder data is converted to joint_states properly") {
    joint_state_received = false;
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, &joint_state_callback);
    auto pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

    // Wait for discovery
    auto start_discovery = node->get_clock()->now();
    while (pub->get_subscription_count() == 0 &&
      (node->get_clock()->now() - start_discovery) < 2s)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }

    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.left_encoder = 1000;
    sensor_data.right_encoder = 1000;

    auto start_time = node->get_clock()->now();
    while (rclcpp::ok() && !joint_state_received &&
      (node->get_clock()->now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
    {
      sensor_data.stamp = node->get_clock()->now();
      pub->publish(sensor_data);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(50ms);
    }

    REQUIRE(joint_state_received);
    REQUIRE(last_joint_state.name.size() == 2);
    // Ensure positions are non-zero positive radians for positive ticks
    CHECK(last_joint_state.position.at(0) > 0.0);
    CHECK(last_joint_state.position.at(1) > 0.0);
  }
}
