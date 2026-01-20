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

/// \brief A robot simulator node with arena walls and cylindrical obstacles.
class Nusimulator : public rclcpp::Node
{
public:
  Nusimulator()
  : Node("nusimulator"), timestep_(0)
  {
    // 1. Basic Parameters
    this->declare_parameter("rate", 100);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);
    this->declare_parameter("arena_x_length", 4.0);
    this->declare_parameter("arena_y_length", 4.0);

    // 2. Obstacle Parameters (Task C.5)
    this->declare_parameter("obstacles.x", std::vector<double>{});
    this->declare_parameter("obstacles.y", std::vector<double>{});
    this->declare_parameter("obstacles.r", 0.1);

    const auto obs_x = this->get_parameter("obstacles.x").as_double_array();
    const auto obs_y = this->get_parameter("obstacles.y").as_double_array();
    const auto obs_r = this->get_parameter("obstacles.r").as_double();

    if (obs_x.size() != obs_y.size()) {
      RCLCPP_FATAL(this->get_logger(), "obstacles.x and obstacles.y must have the same length!");
      throw std::runtime_error("Parameter size mismatch");
    }

    // 3. Setup Publishers with Transient Local QoS
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    wall_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", latched_qos);
    obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", latched_qos);

    // 4. Initialization
    reset_to_initial_pose();
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    int rate_val = this->get_parameter("rate").as_int();
    if (rate_val <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid rate!");
    }
    auto period = std::chrono::milliseconds(1000 / rate_val);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / rate_val),
      std::bind(&Nusimulator::timer_callback, this));

    // Initial Publication
    publish_walls();
    publish_obstacles(obs_x, obs_y, obs_r);

    RCLCPP_INFO(this->get_logger(), "Nusimulator initialized with %zu obstacles.", obs_x.size());
  }

private:
  void reset_to_initial_pose()
  {
    double x = this->get_parameter("x0").as_double();
    double y = this->get_parameter("y0").as_double();
    double th = this->get_parameter("theta0").as_double();
    current_pose_ = turtlelib::Transform2D({x, y}, th);
    timestep_ = 0;
  }

  /// \brief Publishes the cylinders as markers in the 'red' namespace
  void publish_obstacles(const std::vector<double> & x, const std::vector<double> & y, double r)
  {
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < x.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "nusim/world";
      m.header.stamp = this->get_clock()->now();
      m.ns = "red"; // Specified namespace
      m.id = i;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = x[i];
      m.pose.position.y = y[i];
      m.pose.position.z = 0.125; // height / 2
      m.scale.x = 2.0 * r; // Diameter
      m.scale.y = 2.0 * r;
      m.scale.z = 0.25;    // Height
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    obs_pub_->publish(ma);
  }

  void publish_walls()
  {
    double x_len = this->get_parameter("arena_x_length").as_double();
    double y_len = this->get_parameter("arena_y_length").as_double();
    double thick = 0.1, height = 0.25;

    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "nusim/world";
      m.header.stamp = this->get_clock()->now();
      m.ns = "walls";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::CUBE;
      if (i < 2) { // N/S
        m.pose.position.x = (i == 0) ? x_len/2.0 + thick/2.0 : -x_len/2.0 - thick/2.0;
        m.scale.x = thick; m.scale.y = y_len + 2*thick;
      } else { // E/W
        m.pose.position.y = (i == 2) ? y_len/2.0 + thick/2.0 : -y_len/2.0 - thick/2.0;
        m.scale.x = x_len + 2*thick; m.scale.y = thick;
      }
      m.pose.position.z = height/2.0; m.scale.z = height;
      m.color.r = 1.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    wall_pub_->publish(ma);
  }

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = current_pose_.translation().x;
    t.transform.translation.y = current_pose_.translation().y;
    tf2::Quaternion q; q.setRPY(0, 0, current_pose_.rotation());
    t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    timestep_pub_->publish(msg);
    timestep_++;
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    reset_to_initial_pose();
    publish_walls();
    const auto ox = this->get_parameter("obstacles.x").as_double_array();
    const auto oy = this->get_parameter("obstacles.y").as_double_array();
    const auto orad = this->get_parameter("obstacles.r").as_double();
    publish_obstacles(ox, oy, orad);
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}