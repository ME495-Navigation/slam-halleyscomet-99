/// \file
/// \brief Integration tests for the odometry node's ROS API.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/// \brief Test case to verify that the odometry node correctly handles ROS interfaces
TEST_CASE("odometry ROS API", "[odometry]") {
  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

  // --- Test Case 1: Initial Pose Service ---
  SECTION("initial_pose service resets robot configuration") {
    auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

    // Ensure the service is available before proceeding
    REQUIRE(client->wait_for_service(5s));

    auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
    request->x = 5.0;
    request->y = 2.0;
    request->theta = 1.57; // ~PI/2

    auto result_future = client->async_send_request(request);

    // Spin to allow the service call to process
    auto ret = rclcpp::spin_until_future_complete(node, result_future, 5s);
    CHECK(ret == rclcpp::FutureReturnCode::SUCCESS);
  }

  // --- Test Case 2: TF Broadcaster ---
  SECTION("odometry node publishes odom to base_footprint transform") {
    // CRITICAL: Reset odometry to (0,0,0) specifically for this section
    // to ensure we test the Identity Transformation requirement.
    auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");
    REQUIRE(client->wait_for_service(5s));
    auto reset_req = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
    reset_req->x = 0.0;
    reset_req->y = 0.0;
    reset_req->theta = 0.0;
    auto reset_future = client->async_send_request(reset_req);
    rclcpp::spin_until_future_complete(node, reset_future, 5s);

    // Setup TF2 buffer and listener to catch the transform published by odometry node
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Publisher for joint states - Odometry node needs joint data to calculate TF
    auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Wait for the odometry node to discover this publisher
    auto start_discovery = node->get_clock()->now();
    while (joint_pub->get_subscription_count() == 0 &&
      (node->get_clock()->now() - start_discovery) < 2s)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }

    // Prepare a mock joint state message
    // Note: Joint states are kept at 0.0 to verify identity transform as requested
    sensor_msgs::msg::JointState js;
    js.name = {"wheel_left_joint", "wheel_right_joint"};
    js.position = {0.0, 0.0};
    js.velocity = {0.0, 0.0};

    bool transform_found = false;
    geometry_msgs::msg::TransformStamped tf_msg;

    auto start_time = node->get_clock()->now();
    const auto timeout = 5s;

    const std::string odom_frame = "odom";
    const std::string body_frame = "base_footprint";

    // Loop until transform is found or timeout is reached
    while (rclcpp::ok() && (node->get_clock()->now() - start_time) < timeout) {
      // Continuously publish joint states with the latest timestamp
      js.header.stamp = node->get_clock()->now();
      joint_pub->publish(js);

      rclcpp::spin_some(node);

      // Attempt to look up the transform from odom to base_footprint
      try {
        if (tf_buffer->canTransform(odom_frame, body_frame, tf2::TimePointZero, 100ms)) {
          tf_msg = tf_buffer->lookupTransform(odom_frame, body_frame, tf2::TimePointZero);
          transform_found = true;
          break;
        }
      } catch (const tf2::TransformException & ex) {
        // Continue waiting if transform is not yet available in the buffer
      }
      std::this_thread::sleep_for(100ms);
    }

    // Assertions to verify the transform existence and frame IDs
    REQUIRE(transform_found);
    CHECK(tf_msg.header.frame_id == odom_frame);
    CHECK(tf_msg.child_frame_id == body_frame);

    // Verify Identity Transformation (Requirement: the transform should be identity)
    CHECK(tf_msg.transform.translation.x == Catch::Approx(0.0).margin(1e-4));
    CHECK(tf_msg.transform.translation.y == Catch::Approx(0.0).margin(1e-4));
    CHECK(tf_msg.transform.rotation.w == Catch::Approx(1.0).margin(1e-4));
  }
}
