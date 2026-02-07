/// \file
/// \brief Node for controlling the turtlebot wheels and tracking joint states.
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheels [m]
///     track_width (double): distance between the wheels [m]
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
#include <cmath>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

/// \brief Translates between ROS standard messages and nuturtlebot custom messages
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Declare parameters with explicit types
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("motor_cmd_max", rclcpp::PARAMETER_INTEGER);
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);

    // Retrieve parameter values
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto track = get_parameter("track_width").as_double();
    motor_max_ = static_cast<int>(get_parameter("motor_cmd_max").as_int());
    motor_scaling_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_scaling_ = get_parameter("encoder_ticks_per_rad").as_double();

    // Ensure required parameters are loaded
    if (radius <= 0.0 || track <= 0.0 || motor_scaling_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid or missing parameters in diff_params.yaml!");
    }

    // If scaling is 0.024 (rad/s per MCU), we need 41.66 (MCU per rad/s)
    if (motor_scaling_ < 1.0) {
      RCLCPP_DEBUG(get_logger(), "Input scaling %f detected as rad/s per tick. Inverting...",
                  motor_scaling_);
      motor_scaling_ = 1.0 / motor_scaling_;
    }

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

    RCLCPP_DEBUG(get_logger(), "Scaling factors: motor=%f, encoder=%f",
                motor_scaling_, encoder_scaling_);
    RCLCPP_INFO(get_logger(), "turtle_control node has been started.");
  }

private:
  /// \brief Callback for cmd_vel: converts twist to wheel commands via inverse kinematics
  /// \param msg - the desired twist (linear.x and angular.z)
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    try {
      // Calculate required wheel velocities in rad/s
      const auto vels = diff_robot_.inverse_kinematics({msg->angular.z, msg->linear.x, 0.0});

      nuturtlebot_msgs::msg::WheelCommands out;

      // Convert rad/s to motor command units (mcu) using std::round to avoid truncation errors
      // and clamp to the maximum motor command allowed
      out.left_velocity = std::clamp(
        static_cast<int>(std::round(vels.left * motor_scaling_)), -motor_max_, motor_max_);
      out.right_velocity = std::clamp(
        static_cast<int>(std::round(vels.right * motor_scaling_)), -motor_max_, motor_max_);

      wheel_pub_->publish(out);
    } catch (const std::logic_error & e) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Twist ignored: " << e.what());
    }
  }

  /// \brief Callback for sensor_data: publishes joint states based on encoder feedback
  /// \param msg - the raw encoder data from the turtlebot
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    // Convert encoder ticks to wheel positions in radians
    const auto l_pos = static_cast<double>(msg->left_encoder) / encoder_scaling_;
    const auto r_pos = static_cast<double>(msg->right_encoder) / encoder_scaling_;

    sensor_msgs::msg::JointState js;
    js.header.stamp = msg->stamp;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {l_pos, r_pos};

    // Publish joint states for visualization and odometry tracking
    joint_pub_->publish(js);

    // Update internal state of the robot model
    diff_robot_.forward_kinematics({l_pos, r_pos});
  }

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
