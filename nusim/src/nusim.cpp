/// \file
/// \brief Main simulator node for the nuturtle robot, managing ground truth and visualization.
///
/// PARAMETERS:
///     rate (int): Frequency of the simulation loop (Hz).
///     x0 (double): Initial x position of the robot.
///     y0 (double): Initial y position of the robot.
///     theta0 (double): Initial orientation of the robot.
///     arena_x_length (double): Length of the arena (x-direction).
///     arena_y_length (double): Length of the arena (y-direction).
///     obstacles.x (std::vector<double>): X coordinates of cylindrical obstacles.
///     obstacles.y (std::vector<double>): Y coordinates of cylindrical obstacles.
///     obstacles.r (double): Radius of all cylindrical obstacles.
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::UInt64): Current simulation timestep.
///     ~/real_walls (visualization_msgs::msg::MarkerArray): Markers representing the arena walls.
///     ~/real_obstacles (visualization_msgs::msg::MarkerArray): Markers for cylindrical obstacles.
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): Resets simulation state and teleports the robot.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;

/// \brief A robot simulator class that tracks ground truth state and arena elements.
class Nusimulator : public rclcpp::Node
{
public:
  /// \brief Construct a new Nusimulator object
  Nusimulator()
  : Node("nusimulator"), timestep_(0)
  {
    // 1. Declare Parameters
    declare_parameter("rate", 100);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 4.0);
    declare_parameter("arena_y_length", 4.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.1);

    const auto obs_x = get_parameter("obstacles.x").as_double_array();
    const auto obs_y = get_parameter("obstacles.y").as_double_array();
    const auto obs_r = get_parameter("obstacles.r").as_double();

    if (obs_x.size() != obs_y.size()) {
      RCLCPP_FATAL(get_logger(), "obstacles.x and obstacles.y must have the same length!");
      throw std::runtime_error("Parameter size mismatch");
    }

    // 2. Setup Publishers (Transient Local for Markers)
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    wall_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", latched_qos);
    obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/real_obstacles", latched_qos);

    // 3. Setup Broadcaster and Services
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 4. Initialize Simulation State
    reset_to_initial_pose();

    // 5. Timer Setup
    const auto rate_val = get_parameter("rate").as_int();
    const auto period = std::chrono::milliseconds(1000 / rate_val);
    timer_ = create_wall_timer(period, std::bind(&Nusimulator::timer_callback, this));

    // 6. Initial Visualization
    publish_walls();
    publish_obstacles(obs_x, obs_y, obs_r);

    RCLCPP_INFO(get_logger(), "Nusimulator initialized with %zu obstacles.", obs_x.size());
  }

private:
  /// \brief Resets robot pose and simulation timestep based on current parameters
  void reset_to_initial_pose()
  {
    const auto x = get_parameter("x0").as_double();
    const auto y = get_parameter("y0").as_double();
    const auto th = get_parameter("theta0").as_double();
    current_pose_ = turtlelib::Transform2D({x, y}, th);
    timestep_ = 0;
  }

  /// \brief Publishes cylindrical obstacles to the visualization topic
  /// \param x vector of x coordinates
  /// \param y vector of y coordinates
  /// \param r radius of obstacles
  void publish_obstacles(const std::vector<double> & x, const std::vector<double> & y, double r)
  {
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < x.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "nusim/world";
      m.header.stamp = get_clock()->now();
      m.ns = "red";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = x.at(i);
      m.pose.position.y = y.at(i);
      m.pose.position.z = 0.125;
      m.scale.x = 2.0 * r;
      m.scale.y = 2.0 * r;
      m.scale.z = 0.25;
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    obs_pub_->publish(ma);
  }

  /// \brief Publishes rectangular arena walls to the visualization topic
  void publish_walls()
  {
    const auto x_len = get_parameter("arena_x_length").as_double();
    const auto y_len = get_parameter("arena_y_length").as_double();
    const double thick = 0.1;
    const double height = 0.25;

    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "nusim/world";
      m.header.stamp = get_clock()->now();
      m.ns = "walls";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::CUBE;
      if (i < 2) { // North/South walls
        m.pose.position.x = (i == 0) ? x_len / 2.0 + thick / 2.0 : -x_len / 2.0 - thick / 2.0;
        m.scale.x = thick;
        m.scale.y = y_len + 2.0 * thick;
      } else { // East/West walls
        m.pose.position.y = (i == 2) ? y_len / 2.0 + thick / 2.0 : -y_len / 2.0 - thick / 2.0;
        m.scale.x = x_len + 2.0 * thick;
        m.scale.y = thick;
      }
      m.pose.position.z = height / 2.0;
      m.scale.z = height;
      m.color.r = 1.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    wall_pub_->publish(ma);
  }

  /// \brief Timer callback for updating ground truth pose and publishing timestep
  void timer_callback()
  {
    // Broadcast Ground Truth TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = current_pose_.translation().x;
    t.transform.translation.y = current_pose_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.rotation());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    // Timestep logic
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    timestep_pub_->publish(msg);
    timestep_++;
  }

  /// \brief Service to reset the simulation state
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    reset_to_initial_pose();
    publish_walls();
    const auto ox = get_parameter("obstacles.x").as_double_array();
    const auto oy = get_parameter("obstacles.y").as_double_array();
    const auto orad = get_parameter("obstacles.r").as_double();
    publish_obstacles(ox, oy, orad);
    RCLCPP_INFO(get_logger(), "Simulation Reset.");
  }

  uint64_t timestep_;
  turtlelib::Transform2D current_pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// \brief Main entry point for the nusimulator node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}
