#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// \brief A robot simulator node that tracks simulation time and state.
class Nusimulator : public rclcpp::Node
{
public:
  Nusimulator()
  : Node("nusimulator"), timestep_(0)
  {
    // 1. Declare and get the 'rate' parameter (default: 100Hz)
    this->declare_parameter("rate", 100);
    int rate_val = this->get_parameter("rate").as_int();
    auto period = std::chrono::milliseconds(1000 / rate_val);

    // 2. Create the publisher for ~/timestep
    // In ROS 2, not adding a leading slash makes it relative to node namespace
    timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // 3. Create the ~/reset service
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 4. Create the main simulation timer
    timer_ = this->create_wall_timer(
      period, std::bind(&Nusimulator::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Nusimulator initialized at %d Hz", rate_val);
  }

private:
  /// \brief Main timer callback that increments the simulation step
  void timer_callback()
  {
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    timestep_pub_->publish(msg);
    timestep_++;
  }

  /// \brief Resets the simulation state (currently only timestep)
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting simulation timestep to 0");
    timestep_ = 0;
  }

  uint64_t timestep_;
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