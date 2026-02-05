/// \file
/// \brief Node for controlling the turtlebot wheels and tracking joint states.
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheels
///     track_width (double): distance between the wheels
///     motor_cmd_max (int): max motor command units (e.g., 265)
///     motor_cmd_per_rad_sec (double): conversion factor from velocity to motor units
///     encoder_ticks_per_rad (double): conversion factor from ticks to radians
/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): wheel velocity commands
///     joint_states (sensor_msgs::msg::JointState): angle and velocity of wheels
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): desired twist velocity
///     sensor_data (nuturtlebot_msgs::msg::SensorData): encoder data from the robot

#include <algorithm>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

/// \brief Node that translates between ROS standard messages and nuturtlebot messages
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Declare and retrieve parameters from diff_params.yaml
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("motor_cmd_max", rclcpp::PARAMETER_INTEGER);
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);

    const auto radius = get_parameter("wheel_radius").get_value<double>();
    const auto track = get_parameter("track_width").get_value<double>();
    motor_max_ = get_parameter("motor_cmd_max").get_value<int>();
    motor_scaling_ = get_parameter("motor_cmd_per_rad_sec").get_value<double>();
    encoder_scaling_ = get_parameter("encoder_ticks_per_rad").get_value<double>();

    // Initialize the kinematics model from turtlelib
    diff_robot_ = turtlelib::DiffDrive(track, radius);

    // Publishers
    wheel_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_callback, this, std::placeholders::_1));
    sensor_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(get_logger(), "turtle_control node has been started.");
  }

private:
  /// \brief Callback for cmd_vel: uses inverse kinematics to publish wheel commands
  /// \param msg - the desired twist
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    try {
      // Use DiffDrive to calculate required wheel velocities
      const auto vels = diff_robot_.inverse_kinematics({msg->angular.z, msg->linear.x, 0.0});

      nuturtlebot_msgs::msg::WheelCommands out;
      // Convert rad/s to motor command units (mcu) and clamp
      out.left_velocity = std::clamp(
        static_cast<int>(vels.left * motor_scaling_), -motor_max_, motor_max_);
      out.right_velocity = std::clamp(
        static_cast<int>(vels.right * motor_scaling_), -motor_max_, motor_max_);

      wheel_pub_->publish(out);
    } catch (const std::logic_error & e) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Twist ignored: " << e.what());
    }
  }

  /// \brief Callback for sensor_data: uses forward kinematics and publishes joint states
  /// \param msg - the encoder data
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    // Convert ticks to radians
    const auto l_pos = static_cast<double>(msg->left_encoder) / encoder_scaling_;
    const auto r_pos = static_cast<double>(msg->right_encoder) / encoder_scaling_;

    sensor_msgs::msg::JointState js;
    js.header.stamp = msg->stamp;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {l_pos, r_pos};

    // Publish current wheel joint states
    joint_pub_->publish(js);

    // Update the internal DiffDrive model for future FK/Odometry tracking
    diff_robot_.forward_kinematics({l_pos, r_pos});
  }

  // Member variables without this-> according to guidelines
  turtlelib::DiffDrive diff_robot_;
  int motor_max_;
  double motor_scaling_;
  double encoder_scaling_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
