#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;

/// \brief A robot simulator node that tracks ground truth state and broadcasts TF frames.
class Nusimulator : public rclcpp::Node
{
public:
  Nusimulator()
  : Node("nusimulator"), timestep_(0)
  {
    // Declare parameters for initial pose
    this->declare_parameter("rate", 100);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);

    // Initialize state from parameters
    reset_to_initial_pose();

    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create publishers and services
    timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Calculate timer period
    int rate_val = this->get_parameter("rate").as_int();
    auto period = std::chrono::milliseconds(1000 / rate_val);

    // Simulation loop timer
    timer_ = this->create_wall_timer(
      period, std::bind(&Nusimulator::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Nusimulator started with Ground Truth frame: red/base_footprint");
  }

private:
  /// \brief Reads pose parameters and resets simulation state
  void reset_to_initial_pose()
  {
    double x = this->get_parameter("x0").as_double();
    double y = this->get_parameter("y0").as_double();
    double th = this->get_parameter("theta0").as_double();
    
    // Set internal state using turtlelib
    current_pose_ = turtlelib::Transform2D({x, y}, th);
    timestep_ = 0;
  }

  /// \brief Publishes timestep and broadcasts ground truth TF
  void timer_callback()
  {
    // Publish timestep
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    timestep_pub_->publish(msg);

    // Broadcast Transform: nusim/world -> red/base_footprint
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    // Translation from Transform2D
    t.transform.translation.x = current_pose_.translation().x;
    t.transform.translation.y = current_pose_.translation().y;
    t.transform.translation.z = 0.0;

    // Rotation from Transform2D (converted to Quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.rotation());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    timestep_++;
  }

  /// \brief Service callback to reset robot and timestep
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    reset_to_initial_pose();
    RCLCPP_INFO(this->get_logger(), "Simulation Reset complete.");
  }

  uint64_t timestep_;
  turtlelib::Transform2D current_pose_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}