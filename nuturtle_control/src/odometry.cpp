/// \file
/// \brief Publishes odometry messages and the odometry transform based on wheel joint states.
///
/// PARAMETERS:
///     body_id (std::string): The name of the robot's body frame (default: base_footprint).
///     odom_id (std::string): The name of the odometry frame (default: odom).
///     wheel_left (std::string): The name of the left wheel joint.
///     wheel_right (std::string): The name of the right wheel joint.
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The distance between the wheels.
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): The robot's estimated pose and velocity.
///     ~/path (nav_msgs::msg::Path): The path the robot has taken according to odometry (Blue).
///     /tf (tf2_msgs::msg::TFMessage): The transform from odom_id to body_id.
/// SUBSCRIBES:
///     joint_states (sensor_msgs::msg::JointState): The positions of the wheels.
/// SERVERS:
///     initial_pose (nuturtle_control::srv::InitialPose): Resets the odometry to a specific pose.

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

namespace nuturtle_control
{
/// \brief Node that computes and publishes robot odometry.
class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode()
  : Node("odometry")
  {
    // 1. Declare and retrieve parameters
    body_id_ = declare_parameter("body_id", "base_footprint");
    odom_id_ = declare_parameter("odom_id", "odom");

    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    wheel_left_name_ = get_parameter("wheel_left").as_string();
    wheel_right_name_ = get_parameter("wheel_right").as_string();

    if (wheel_left_name_.empty() || wheel_right_name_.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_left and wheel_right must be specified!");
      rclcpp::shutdown();
      return;
    }

    const auto radius = declare_parameter("wheel_radius", 0.033);
    const auto track = declare_parameter("track_width", 0.16);

    // Initialize the kinematics model from turtlelib
    diff_robot_ = turtlelib::DiffDrive(track, radius);

    // 2. Initialize ROS interfaces
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Initialize Path Publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    path_msg_.header.frame_id = odom_id_;

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&OdometryNode::joint_callback, this, std::placeholders::_1));

    initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &OdometryNode::initial_pose_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(get_logger(), "Odometry node initialized with Path visualization.");
  }

private:
  /// \brief Callback for JointState messages to update odometry.
  /// \param msg - The incoming JointState message.
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    int left_idx = -1;
    int right_idx = -1;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name.at(i) == wheel_left_name_) {
        left_idx = static_cast<int>(i);
      }
      if (msg->name.at(i) == wheel_right_name_) {
        right_idx = static_cast<int>(i);
      }
    }

    if (left_idx != -1 && right_idx != -1) {
      double left_pos = msg->position.at(left_idx);
      double right_pos = msg->position.at(right_idx);

      // If this is the first update or after a reset, record the current position as offset
      if (sync_wheels_) {
        left_offset_ = left_pos;
        right_offset_ = right_pos;
        sync_wheels_ = false;
      }

      // Update internal model using positions relative to the reset/start point
      diff_robot_.forward_kinematics(
        {left_pos - left_offset_,
          right_pos - right_offset_});

      publish_odom(msg->header.stamp);
    }
  }

  /// \brief Broadcasts TF, publishes Odometry and Path messages.
  /// \param stamp - The timestamp for the messages.
  void publish_odom(const rclcpp::Time & stamp)
  {
    const auto q = diff_robot_.configuration();

    // 1. Orientation from Euler to Quaternion
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, q.rotation());

    // 2. Broadcast Transform between odom_id and body_id
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_id_;
    tf_msg.child_frame_id = body_id_;
    tf_msg.transform.translation.x = q.translation().x;
    tf_msg.transform.translation.y = q.translation().y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = quat.x();
    tf_msg.transform.rotation.y = quat.y();
    tf_msg.transform.rotation.z = quat.z();
    tf_msg.transform.rotation.w = quat.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // 3. Prepare and publish Odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_id_;
    odom.child_frame_id = body_id_;
    odom.pose.pose.position.x = q.translation().x;
    odom.pose.pose.position.y = q.translation().y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom_pub_->publish(odom);

    // 4. Update and Publish Path (Blue Path)
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = odom_id_;
    ps.pose = odom.pose.pose;
    path_msg_.poses.push_back(ps);
    path_msg_.header.stamp = stamp;
    path_pub_->publish(path_msg_);
  }

  /// \brief Resets the robot's configuration via service and clears the path.
  /// \param request - The desired x, y, theta.
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    // Reset the kinematic model to the new starting pose
    diff_robot_ = turtlelib::DiffDrive(
      diff_robot_.track_width(),
      diff_robot_.wheel_radius(),
      turtlelib::Transform2D({request->x, request->y}, request->theta)
    );
    
    // Set flag to sync wheel offsets on next joint_state message
    sync_wheels_ = true;

    // Clear the path when resetting pose
    path_msg_.poses.clear();
    
    RCLCPP_INFO_STREAM(get_logger(), "Odometry and Path reset. Waiting for next joint_state to sync wheels.");
  }

  // Member variables
  std::string body_id_, odom_id_, wheel_left_name_, wheel_right_name_;
  turtlelib::DiffDrive diff_robot_{0.16, 0.033};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;

  // Synchronization variables
  double left_offset_ = 0.0;
  double right_offset_ = 0.0;
  bool sync_wheels_ = true;
};
}  // namespace nuturtle_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nuturtle_control::OdometryNode>());
  rclcpp::shutdown();
  return 0;
}