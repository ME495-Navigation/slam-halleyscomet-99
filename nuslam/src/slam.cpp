/// \file
/// \brief Main SLAM node for performing Feature-Based EKF SLAM.
///
/// PARAMETERS:
///     body_id (std::string): The frame ID for the green robot body (green/base_footprint).
///     odom_id (std::string): The frame ID for the odometry frame (odom).
///     map_id (std::string): The frame ID for the map frame (map).
///     wheel_radius (double): Radius of the robot wheels [m].
///     track_width (double): Distance between the wheels [m].
/// PUBLISHES:
///     ~/path (nav_msgs::msg::Path): Corrected SLAM trajectory (Green) in map frame.
///     ~/odom_path (nav_msgs::msg::Path): Pure accumulated trajectory (Blue) in world frame.
///     ~/slam_obstacles (visualization_msgs::msg::MarkerArray): SLAM estimated landmarks.
///     green/joint_states (sensor_msgs::msg::JointState): Joint states for green robot model.
/// SUBSCRIBES:
///     /red/joint_states (sensor_msgs::msg::JointState): Source of encoder data for prediction.
///     /fake_sensor (visualization_msgs::msg::MarkerArray): Relative landmark measurements.
///     /nusimulator/real_walls (visualization_msgs::msg::MarkerArray): Simulated wall markers.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/ekf.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/// \brief A node implementing EKF SLAM with dual-path comparison in a static world frame.
/// \details Manages independent visualization branches. The Green robot is corrected by SLAM
/// measurements via the map->odom transform, while the Blue robot uses pure odometry
/// anchored directly to 'nusim/world' to visualize encoder drift.
class SlamNode : public rclcpp::Node
{
public:
  /// \brief Constructor for the SlamNode.
  /// \details Initializes EKF state, kinematic models, parameters, and ROS interfaces.
  SlamNode()
  : Node("slam"), ekf_(20)
  {
    // 1. Declare and retrieve parameters
    declare_parameter("body_id", "green/base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter("map_id", "map");
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);

    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    map_id_ = get_parameter("map_id").as_string();
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto track = get_parameter("track_width").as_double();

    // 2. Initialize kinematic models
    diff_robot_ = turtlelib::DiffDrive(track, radius);

    // 3. Setup Subscriptions
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/red/joint_states", 10, std::bind(&SlamNode::joint_callback, this, std::placeholders::_1));

    landmark_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10, std::bind(&SlamNode::landmark_callback, this, std::placeholders::_1));

    wall_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/nusimulator/real_walls", 10,
      std::bind(&SlamNode::wall_callback, this, std::placeholders::_1));

    // 4. Setup Publishers
    path_pub_green_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    path_pub_blue_ = create_publisher<nav_msgs::msg::Path>("~/odom_path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/slam_obstacles", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("green/joint_states", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Set static frame IDs for Path headers
    slam_path_msg_.header.frame_id = map_id_;
    odom_path_msg_.header.frame_id = "nusim/world";

    RCLCPP_INFO(get_logger(), "SLAM Node initialized. Fixed Frame: nusim/world");
  }

private:
  /// \brief Callback for encoder data (EKF Prediction Step).
  /// \details Calculates twist for EKF and absolute configuration for pure odometry comparison.
  /// \param msg - Hardware wheel encoder positions.
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const double left = msg->position.at(0);
    const double right = msg->position.at(1);
    const rclcpp::Time current_stamp = msg->header.stamp;

    if (first_joint_) {
      left_prev_ = left;
      right_prev_ = right;
      last_joint_time_ = current_stamp;
      first_joint_ = false;
      return;
    }

    // Filter frequency jitter to maintain sync with 100Hz simulator physics
    double dt = (current_stamp - last_joint_time_).seconds();
    if (dt < 0.005) {return;}

    // Calculate wheel increments
    const double dphi_l = left - left_prev_;
    const double dphi_r = right - right_prev_;
    const auto r = diff_robot_.wheel_radius();
    const auto d = diff_robot_.track_width();

    // Body Frame increments for EKF prediction step
    const double dx = (r / 2.0) * (dphi_l + dphi_r);
    const double dtheta = (r / d) * (dphi_r - dphi_l);

    // 1. Predict SLAM state via EKF
    ekf_.predict(turtlelib::Transform2D({dx, 0.0}, dtheta));

    // 2. Update Pure Odometry (Independent Ghost Branch)
    diff_robot_.forward_kinematics({left, right});
    odom_pose_ = diff_robot_.configuration();

    // Update persistence variables
    left_prev_ = left;
    right_prev_ = right;
    last_joint_time_ = current_stamp;

    // Trigger publishing synchronized to encoder data
    publish_data(current_stamp);
  }

  /// \brief Callback for EKF Correction Step using known data association.
  /// \param msg - Landmark measurements relative to the robot footprint.
  void landmark_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (const auto & marker : msg->markers) {
      if (marker.action == visualization_msgs::msg::Marker::ADD) {
        ekf_.update(marker.id, marker.pose.position.x, marker.pose.position.y);
      }
    }
  }

  /// \brief Stores simulator walls for visualization synchronization.
  /// \param msg - Walls published by the nusimulator.
  void wall_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    slam_walls_ = *msg;
  }

  /// \brief Manages TF tree and publishes synchronized visualization data for Green and Blue robots.
  /// \param stamp - The hardware timestamp derived from the JointState header.
  void publish_data(const rclcpp::Time & stamp)
  {
    const auto slam_pose = ekf_.estimate_pose();
    const auto current_odom = odom_pose_;
    const auto T_mo = slam_pose * current_odom.inv();

    // ---------------------------------------------------------
    // BRANCH 1: SLAM CORRECTED ROBOT (GREEN)
    // ---------------------------------------------------------

    // 1. Map -> Odom TF (The SLAM correction term)
    geometry_msgs::msg::TransformStamped tf_mo;
    tf_mo.header.stamp = stamp;
    tf_mo.header.frame_id = map_id_;
    tf_mo.child_frame_id = odom_id_;
    tf_mo.transform.translation.x = T_mo.translation().x;
    tf_mo.transform.translation.y = T_mo.translation().y;
    tf_mo.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0, 0, T_mo.rotation());
    tf_mo.transform.rotation.x = q.x(); tf_mo.transform.rotation.y = q.y();
    tf_mo.transform.rotation.z = q.z(); tf_mo.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_mo);

    // 2. Odom -> green/base_footprint TF (The SLAM-estimated footprint)
    geometry_msgs::msg::TransformStamped tf_green;
    tf_green.header.stamp = stamp;
    tf_green.header.frame_id = odom_id_;
    tf_green.child_frame_id = body_id_;
    tf_green.transform.translation.x = current_odom.translation().x;
    tf_green.transform.translation.y = current_odom.translation().y;
    tf_green.transform.translation.z = 0.0;
    tf2::Quaternion q_odom; q_odom.setRPY(0, 0, current_odom.rotation());
    tf_green.transform.rotation.x = q_odom.x(); tf_green.transform.rotation.y = q_odom.y();
    tf_green.transform.rotation.z = q_odom.z(); tf_green.transform.rotation.w = q_odom.w();
    tf_broadcaster_->sendTransform(tf_green);

    // 3. SLAM Path (Corrected, anchored to map frame)
    slam_path_msg_.header.stamp = stamp;
    geometry_msgs::msg::PoseStamped slam_ps;
    slam_ps.header.stamp = stamp;
    slam_ps.header.frame_id = map_id_;
    slam_ps.pose.position.x = slam_pose.translation().x;
    slam_ps.pose.position.y = slam_pose.translation().y;
    tf2::Quaternion q_slam; q_slam.setRPY(0, 0, slam_pose.rotation());
    slam_ps.pose.orientation.x = q_slam.x(); slam_ps.pose.orientation.y = q_slam.y();
    slam_ps.pose.orientation.z = q_slam.z(); slam_ps.pose.orientation.w = q_slam.w();
    slam_path_msg_.poses.push_back(slam_ps);
    path_pub_green_->publish(slam_path_msg_);


    // ---------------------------------------------------------
    // BRANCH 2: PURE ODOMETRY ROBOT
    // ---------------------------------------------------------

    // 4. nusim/world -> blue/base_footprint TF
    // Bypasses the map->odom correction shift to allow wall-crossing visualization.
    geometry_msgs::msg::TransformStamped tf_blue;
    tf_blue.header.stamp = stamp;
    tf_blue.header.frame_id = "nusim/world";
    tf_blue.child_frame_id = "blue/base_footprint";
    tf_blue.transform.translation.x = current_odom.translation().x;
    tf_blue.transform.translation.y = current_odom.translation().y;
    tf_blue.transform.translation.z = 0.0;
    tf_blue.transform.rotation = tf_green.transform.rotation;
    tf_broadcaster_->sendTransform(tf_blue);

    // 5. Blue Path (Pure drift, anchored to static nusim/world frame)
    odom_path_msg_.header.stamp = stamp;
    geometry_msgs::msg::PoseStamped odom_ps;
    odom_ps.header.stamp = stamp;
    odom_ps.header.frame_id = "nusim/world";
    odom_ps.pose.position.x = tf_blue.transform.translation.x;
    odom_ps.pose.position.y = tf_blue.transform.translation.y;
    odom_ps.pose.orientation = tf_blue.transform.rotation;
    odom_path_msg_.poses.push_back(odom_ps);
    path_pub_blue_->publish(odom_path_msg_);


    // ---------------------------------------------------------
    // SYSTEM OUTPUTS
    // ---------------------------------------------------------

    // 6. Joint States for Green robot model wheel rotations
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"wheel_left_joint", "wheel_right_joint"};
    js.position = {left_prev_, right_prev_};
    joint_pub_->publish(js);

    // 7. SLAM Estimated Landmarks (Green Cylinders)
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < 20; ++i) {
      const auto lm = ekf_.estimate_landmark(i);
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_id_; m.header.stamp = stamp;
      m.ns = "slam_landmarks"; m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = (std::abs(lm(0)) < 1e-5 && std::abs(lm(1)) < 1e-5) ?
        visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = lm(0); m.pose.position.y = lm(1); m.pose.position.z = 0.125;
      m.scale.x = 0.076; m.scale.y = 0.076; m.scale.z = 0.25;
      m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    marker_pub_->publish(ma);
  }

  // --- Private Members ---
  std::string body_id_, odom_id_, map_id_;
  double left_prev_ = 0.0, right_prev_ = 0.0;
  bool first_joint_ = true;
  rclcpp::Time last_joint_time_;

  turtlelib::Ekf ekf_;
  turtlelib::DiffDrive diff_robot_{0.0, 0.0};
  turtlelib::Transform2D odom_pose_;
  nav_msgs::msg::Path slam_path_msg_, odom_path_msg_;
  visualization_msgs::msg::MarkerArray slam_walls_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr wall_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_green_, path_pub_blue_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

/// \brief Entry point for the SLAM node.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}
