/// \file
/// \brief Publishes cmd_vel to make the robot drive in a circle.
///
/// PARAMETERS:
///     frequency (double): Frequency of publishing cmd_vel (default: 100.0).
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): Velocity command.
/// SERVERS:
///     control (nuturtle_control::srv::Control): Sets velocity and radius.
///     reverse (std_srvs::srv::Empty): Reverses the current direction.
///     stop (std_srvs::srv::Empty): Stops the robot.

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;

namespace nuturtle_control
{
/// \brief Node that commands a robot to move in a circle.
class CircleNode : public rclcpp::Node
{
public:
  CircleNode()
  : Node("circle")
  {
    // 1. Parameters
    const auto frequency = declare_parameter("frequency", 100.0);
    const auto period = std::chrono::duration<double>(1.0 / frequency);

    // 2. Publishers
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 3. Services
    control_srv_ = create_service<nuturtle_control::srv::Control>(
      "control",
        std::bind(&CircleNode::control_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    reverse_srv_ = create_service<std_srvs::srv::Empty>(
      "reverse",
        std::bind(&CircleNode::reverse_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Empty>(
      "stop",
        std::bind(&CircleNode::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 4. Timer for constant publication
    timer_ = create_wall_timer(period, std::bind(&CircleNode::timer_callback, this));

    RCLCPP_INFO_STREAM(get_logger(), "Circle node started at " << frequency << "Hz.");
  }

private:
  /// \brief Fixed rate timer callback to publish cmd_vel
  void timer_callback()
  {
    geometry_msgs::msg::Twist msg;
    if (active_) {
      msg.linear.x = velocity_ * radius_;
      msg.angular.z = velocity_;
    } else {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }
    cmd_pub_->publish(msg);
  }

  /// \brief Service to update circle parameters
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    velocity_ = request->velocity;
    radius_ = request->radius;
    active_ = true;
    RCLCPP_INFO_STREAM(get_logger(), "Moving in circle: v=" << velocity_ << " r=" << radius_);
  }

  /// \brief Service to reverse direction
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    velocity_ = -velocity_;
    RCLCPP_INFO_STREAM(get_logger(), "Direction reversed.");
  }

  /// \brief Service to stop movement
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    active_ = false;
    RCLCPP_INFO_STREAM(get_logger(), "Robot stopped.");
  }

  // Variables
  double velocity_ = 0.0;
  double radius_ = 0.0;
  bool active_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace nuturtle_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nuturtle_control::CircleNode>());
  rclcpp::shutdown();
  return 0;
}
