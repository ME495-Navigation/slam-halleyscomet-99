/// \file
/// \brief Integration test to verify that the circle node publishes at the correct frequency.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

using namespace std::chrono_literals;

// Global counter for received messages
int message_count = 0;

/// \brief Callback function for the cmd_vel subscriber
/// \param msg - The received Twist message
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  (void)msg; // Suppress unused parameter warning
  message_count++;
}

TEST_CASE("circle node frequency", "[circle_integration]") {
  auto node = rclcpp::Node::make_shared("turtle_circle_test_node");

  // Declare test_duration parameter to allow the launch file to control test length
  node->declare_parameter<double>("test_duration", 2.0);
  const auto TEST_DURATION = node->get_parameter("test_duration").as_double();

  // Create subscriber to the cmd_vel topic
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, &cmd_vel_callback);

  // Record start time using the ROS clock
  auto start_time = node->get_clock()->now();

  // Run the loop for the duration specified by the parameter
  while (rclcpp::ok() &&
    (node->get_clock()->now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    rclcpp::spin_some(node);
  }

  // Calculate the expected number of messages based on 100Hz frequency
  // Frequency = count / duration -> count = frequency * duration
  const double expected_hz = 100.0;
  const double expected_count = expected_hz * TEST_DURATION;

  // Verify that the number of messages received is within a 10% tolerance
  // This accounts for startup delays and system jitter
  // 100Hz at 2.0s = 200 messages. 10% margin allows 180-220.
  REQUIRE_THAT(static_cast<double>(message_count),
               Catch::Matchers::WithinRel(expected_count, 0.1));
}
