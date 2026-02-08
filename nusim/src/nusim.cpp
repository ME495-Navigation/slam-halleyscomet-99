/// \file
/// \brief Main simulator node for the nuturtle robot, managing ground truth and visualization.
///
/// PARAMETERS:
///     rate (int): Frequency of the simulation loop (Hz).
///     x0 (double): Initial x position of the robot [m].
///     y0 (double): Initial y position of the robot [m].
///     theta0 (double): Initial orientation of the robot [rad].
///     arena_x_length (double): Length of the arena in the x-direction [m].
///     arena_y_length (double): Length of the arena in the y-direction [m].
///     obstacles.x (std::vector<double>): X coordinates of cylindrical obstacles [m].
///     obstacles.y (std::vector<double>): Y coordinates of cylindrical obstacles [m].
///     obstacles.r (double): Radius of all cylindrical obstacles [m].
///     motor_cmd_per_rad_sec (double): Scaling factor for motor commands to rad/s.
///     encoder_ticks_per_rad (double): Ticks per radian for the encoders.
///     wheel_radius (double): Radius of the robot wheels [m].
///     track_width (double): Distance between wheels [m].
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::UInt64): Current simulation timestep count.
///     ~/real_walls (visualization_msgs::msg::MarkerArray): Markers representing the arena walls.
///     ~/real_obstacles (visualization_msgs::msg::MarkerArray): Markers for cylindrical obstacles.
///     red/sensor_data (nuturtlebot_msgs::msg::SensorData): Simulated encoder readings.
///     red/joint_states (sensor_msgs::msg::JointState): The joint positions of the red robot.
/// SUBSCRIBES:
///     red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Commands to set wheel velocities.
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): Resets simulation state to initial configuration.

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
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/// \brief A robot simulator class that tracks ground truth state and arena elements.
class Nusimulator : public rclcpp::Node
{
public:
  /// \brief Construct a new Nusimulator object and initialize ROS parameters and interfaces.
  Nusimulator()
  : Node("nusimulator"), timestep_(0)
  {
    // 1. Declare and Retrieve Parameters
    declare_parameter("rate", 100);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 4.0);
    declare_parameter("arena_y_length", 4.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.1);

    // Kinematics/Physics Parameters
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.47);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);

    const auto obs_x = get_parameter("obstacles.x").as_double_array();
    const auto obs_y = get_parameter("obstacles.y").as_double_array();
    const auto obs_r = get_parameter("obstacles.r").as_double();
    motor_scaling_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_scaling_ = get_parameter("encoder_ticks_per_rad").as_double();
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto track = get_parameter("track_width").as_double();

    if (obs_x.size() != obs_y.size()) {
      RCLCPP_FATAL(get_logger(), "obstacles.x and obstacles.y must have the same length!");
      throw std::runtime_error("Parameter size mismatch");
    }

    // 2. Setup Publishers/Subscribers
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    wall_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", latched_qos);
    obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/real_obstacles", latched_qos);

    // Feed simulated hardware data to turtle_control
    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

    // Publish JointStates so robot_state_publisher can animate the wheels
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

    // Subscribe to wheel commands from turtle_control
    wheel_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, std::bind(&Nusimulator::wheel_cmd_callback, this, std::placeholders::_1));

    // 3. Setup Broadcaster and Services
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 4. Initialize Simulation State
    diff_robot_ = turtlelib::DiffDrive(track, radius);
    reset_to_initial_pose();

    // 5. Timer Setup
    const auto rate_val = get_parameter("rate").as_int();
    dt_ = 1.0 / static_cast<double>(rate_val);
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 * dt_));
    timer_ = create_wall_timer(period, std::bind(&Nusimulator::timer_callback, this));

    // 6. Initial Visualization
    publish_walls();
    publish_obstacles(obs_x, obs_y, obs_r);

    RCLCPP_INFO(get_logger(), "Nusimulator initialized.");
  }

private:
  /// \brief Callback for incoming wheel velocity commands in MCU units.
  /// \param msg - The wheel command message containing MCU velocities.
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    left_wheel_vel_ = static_cast<double>(msg->left_velocity) * motor_scaling_;
    right_wheel_vel_ = static_cast<double>(msg->right_velocity) * motor_scaling_;
  }

  /// \brief Resets robot to initial pose parameters and clears encoders/velocities.
  void reset_to_initial_pose()
  {
    const auto x = get_parameter("x0").as_double();
    const auto y = get_parameter("y0").as_double();
    const auto th = get_parameter("theta0").as_double();

    current_pose_ = turtlelib::Transform2D({x, y}, th);
    diff_robot_ = turtlelib::DiffDrive(
      diff_robot_.track_width(), diff_robot_.wheel_radius(), current_pose_);

    left_wheel_pos_ = 0.0;
    right_wheel_pos_ = 0.0;
    left_wheel_vel_ = 0.0;
    right_wheel_vel_ = 0.0;
    timestep_ = 0;
  }

  /// \brief Main loop: Updates physics, publishes sensor data, TF, and joint states.
  void timer_callback()
  {
    const auto current_time = get_clock()->now();

    // 1. Update Physics (Euler integration of wheel positions)
    left_wheel_pos_ += left_wheel_vel_ * dt_;
    right_wheel_pos_ += right_wheel_vel_ * dt_;

    // Update the kinematics model and get ground truth pose
    diff_robot_.forward_kinematics({left_wheel_pos_, right_wheel_pos_});
    current_pose_ = diff_robot_.configuration();

    // 2. Publish SensorData (Ticks for turtle_control)
    nuturtlebot_msgs::msg::SensorData sd;
    sd.stamp = current_time;
    sd.left_encoder = static_cast<int32_t>(left_wheel_pos_ * encoder_scaling_);
    sd.right_encoder = static_cast<int32_t>(right_wheel_pos_ * encoder_scaling_);
    sensor_pub_->publish(sd);

    // 3. Publish JointState (Radian positions for robot_state_publisher)
    sensor_msgs::msg::JointState js;
    js.header.stamp = current_time;
    js.name = {"wheel_left_joint", "wheel_right_joint"};
    js.position = {left_wheel_pos_, right_wheel_pos_};
    joint_pub_->publish(js);

    // 4. Broadcast Ground Truth TF (nusim/world -> red/base_footprint)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
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

    // 5. Publish internal timestep count
    std_msgs::msg::UInt64 ts_msg;
    ts_msg.data = timestep_;
    timestep_pub_->publish(ts_msg);
    timestep_++;
  }

  /// \brief Create and publish cylindrical markers for obstacles.
  void publish_obstacles(const std::vector<double> & x, const std::vector<double> & y, double r)
  {
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < x.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "nusim/world";
      m.header.stamp = get_clock()->now();
      m.ns = "real_obstacles";
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

  /// \brief Create and publish markers for arena boundaries.
  void publish_walls()
  {
    const auto x_len = get_parameter("arena_x_length").as_double();
    const auto y_len = get_parameter("arena_y_length").as_double();
    const double thick = 0.01;
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

  /// \brief Service callback to reset the simulation state.
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
  double dt_;
  double motor_scaling_, encoder_scaling_;
  double left_wheel_pos_ = 0.0, right_wheel_pos_ = 0.0;
  double left_wheel_vel_ = 0.0, right_wheel_vel_ = 0.0;

  turtlelib::Transform2D current_pose_;
  turtlelib::DiffDrive diff_robot_{0.0, 0.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// \brief Entry point for the nusimulator node.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}
