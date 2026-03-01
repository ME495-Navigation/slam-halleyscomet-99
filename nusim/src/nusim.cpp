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
///     laser_max_range (double): Max range of simulated laser.
///     draw_only (bool): If true, only draw landmarks and walls, no simulation.
///     input_noise (double): Variance of the zero-mean Gaussian noise on motor commands.
///     slip_fraction (double): Range of the uniform random fraction for wheel slip.
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::UInt64): Current simulation timestep count.
///     ~/real_walls (visualization_msgs::msg::MarkerArray): Markers representing the arena walls.
///     ~/real_obstacles (visualization_msgs::msg::MarkerArray): Markers for cylindrical obstacles.
///     ~/seen_obstacles (visualization_msgs::msg::MarkerArray): Yellow markers seen by the robot.
///     ~/path (nav_msgs::msg::Path): Red path tracing the robot ground truth.
///     ~/laser_scan (sensor_msgs::msg::LaserScan): Simulated laser scan data.
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
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
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
    declare_parameter("laser_max_range", 3.0);
    declare_parameter("draw_only", false);

    // Noise Parameters
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);

    // Kinematics/Physics Parameters
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.47);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 3.0);

    obs_r_ = get_parameter("obstacles.r").as_double();
    laser_max_range_ = get_parameter("laser_max_range").as_double();
    motor_scaling_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_scaling_ = get_parameter("encoder_ticks_per_rad").as_double();
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto track = get_parameter("track_width").as_double();
    draw_only_ = get_parameter("draw_only").as_bool();
    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();

    // Initialize sensor noise distribution
    sensor_dist_ = std::normal_distribution<double>(0.0, std::sqrt(basic_sensor_variance_));

    // Initialize Random Engine and Distributions
    rng_.seed(std::random_device{}());
    gaussian_dist_ = std::normal_distribution<double>(0.0, std::sqrt(input_noise_));
    slip_dist_ = std::uniform_real_distribution<double>(-slip_fraction_, slip_fraction_);

    // 2. Setup Publishers
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    wall_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", latched_qos);
    obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/real_obstacles", latched_qos);

    // 3. Conditional Setup
    if (draw_only_) {
      timer_ = create_wall_timer(500ms, std::bind(&Nusimulator::draw_only_timer_callback, this));
      RCLCPP_INFO(get_logger(), "Nusimulator started in DRAW_ONLY mode (nuwalls).");
    } else {
      timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      seen_obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/seen_obstacles", 10);
      path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
      scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("~/laser_scan", 10);
      sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

      fake_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/fake_sensor", 10);

      // 5 Hz timer for the fake sensor (200ms period)
      sensor_timer_ = create_wall_timer(200ms, std::bind(&Nusimulator::sensor_timer_callback, this));

      wheel_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd", 10, std::bind(&Nusimulator::wheel_cmd_callback, this, std::placeholders::_1));

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      reset_srv_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&Nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

      diff_robot_ = turtlelib::DiffDrive(track, radius);
      path_msg_.header.frame_id = "nusim/world";
      reset_to_initial_pose();

      const auto rate_val = get_parameter("rate").as_int();
      dt_ = 1.0 / static_cast<double>(rate_val);
      const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 * dt_));
      timer_ = create_wall_timer(period, std::bind(&Nusimulator::timer_callback, this));
    }
  }

private:
  /// \brief Timer callback for draw_only mode.
  void draw_only_timer_callback()
  {
    publish_walls();
    const auto ox = get_parameter("obstacles.x").as_double_array();
    const auto oy = get_parameter("obstacles.y").as_double_array();
    publish_obstacles(ox, oy, obs_r_);
  }

  /// \brief Callback for the 5Hz sensor timer. Calculates relative obstacle positions with noise.
  void sensor_timer_callback()
  {
    const auto ox = get_parameter("obstacles.x").as_double_array();
    const auto oy = get_parameter("obstacles.y").as_double_array();
    const auto current_time = get_clock()->now();
    
    visualization_msgs::msg::MarkerArray ma;

    for (size_t i = 0; i < ox.size(); ++i) {
      // 1. Calculate relative position in world coordinates
      double dx = ox[i] - current_pose_.translation().x;
      double dy = oy[i] - current_pose_.translation().y;

      // 2. Transform to robot's local frame
      double th = current_pose_.rotation();
      double local_x = dx * std::cos(th) + dy * std::sin(th);
      double local_y = -dx * std::sin(th) + dy * std::cos(th);
      double dist = std::sqrt(local_x * local_x + local_y * local_y);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "red/base_footprint"; // Relative to robot
      m.header.stamp = current_time;
      m.ns = "fake_sensor";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CYLINDER;

      // 3. Range check
      if (dist <= max_range_) {
        m.action = visualization_msgs::msg::Marker::ADD;
        // Add zero-mean Gaussian noise to the measured coordinates
        m.pose.position.x = local_x + sensor_dist_(rng_);
        m.pose.position.y = local_y + sensor_dist_(rng_);
        m.pose.position.z = 0.125;
        
        m.scale.x = 2.0 * obs_r_;
        m.scale.y = 2.0 * obs_r_;
        m.scale.z = 0.25;
        // Marker color (different from real obstacles to distinguish)
        m.color.r = 0.5; m.color.g = 0.0; m.color.b = 0.5; m.color.a = 1.0; 
      } else {
        // Set action to DELETE if out of range
        m.action = visualization_msgs::msg::Marker::DELETE;
      }
      ma.markers.push_back(m);
    }
    fake_sensor_pub_->publish(ma);
  }

  /// \brief Callback for wheel commands: Stores raw commands for the main loop.
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    u_l_cmd_ = static_cast<double>(msg->left_velocity) * motor_scaling_;
    u_r_cmd_ = static_cast<double>(msg->right_velocity) * motor_scaling_;
  }

  /// \brief Main loop: Computes unique noise/slip per frame and updates physics.
  void timer_callback()
  {
    const auto current_time = get_clock()->now();
    
    // Step 2 & 3: Compute v_i = u_i + w_i (Gaussian noise per frame)
    auto get_v_sensor = [&](double u) {
      if (std::abs(u) < 1e-6) return 0.0; // motors at rest remain at rest
      return u + gaussian_dist_(rng_); // unique noise for this wheel this frame
    };

    v_l_sensor_ = get_v_sensor(u_l_cmd_);
    v_r_sensor_ = get_v_sensor(u_r_cmd_);

    // Step 4: Compute physical velocity with slip (Uniform slip per frame)
    v_l_physics_ = v_l_sensor_ * (1.0 + slip_dist_(rng_));
    v_r_physics_ = v_r_sensor_ * (1.0 + slip_dist_(rng_));

    // 1. Update Physics Layer (Ground Truth - With Slip)
    left_wheel_pos_ += v_l_physics_ * dt_;
    right_wheel_pos_ += v_r_physics_ * dt_;
    diff_robot_.forward_kinematics({left_wheel_pos_, right_wheel_pos_});
    current_pose_ = diff_robot_.configuration();

    // 2. Update Sensor Layer (Encoder reading - Without Slip)
    left_encoder_pos_ += v_l_sensor_ * dt_;
    right_encoder_pos_ += v_r_sensor_ * dt_;

    // 3. Publish Robot State and TF
    publish_robot_state(current_time);

    // 4. Publish Visualization
    publish_path(current_time);
    publish_seen_markers(current_time);
    publish_laser_scan(current_time);
    
    if (timestep_ % 50 == 0) {
      publish_walls();
      const auto ox = get_parameter("obstacles.x").as_double_array();
      const auto oy = get_parameter("obstacles.y").as_double_array();
      publish_obstacles(ox, oy, obs_r_);
    }

    // 5. Timestep
    std_msgs::msg::UInt64 ts_msg;
    ts_msg.data = timestep_;
    timestep_pub_->publish(ts_msg);
    timestep_++;
  }

  /// \brief Resets robot to initial pose and clears telemetry.
  void reset_to_initial_pose()
  {
    const auto x = get_parameter("x0").as_double();
    const auto y = get_parameter("y0").as_double();
    const auto th = get_parameter("theta0").as_double();

    current_pose_ = turtlelib::Transform2D({x, y}, th);
    diff_robot_ = turtlelib::DiffDrive(
      diff_robot_.track_width(), diff_robot_.wheel_radius(), current_pose_);

    left_wheel_pos_ = 0.0; right_wheel_pos_ = 0.0;
    left_encoder_pos_ = 0.0; right_encoder_pos_ = 0.0;
    u_l_cmd_ = 0.0; u_r_cmd_ = 0.0;
    v_l_physics_ = 0.0; v_r_physics_ = 0.0;
    v_l_sensor_ = 0.0; v_r_sensor_ = 0.0;
    timestep_ = 0;
    path_msg_.poses.clear();
  }

  void publish_path(const rclcpp::Time & current_time)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = current_time;
    ps.header.frame_id = "nusim/world";
    ps.pose.position.x = current_pose_.translation().x;
    ps.pose.position.y = current_pose_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.rotation());
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    path_msg_.poses.push_back(ps);
    path_msg_.header.stamp = current_time;
    path_pub_->publish(path_msg_);
  }

  void publish_robot_state(const rclcpp::Time & current_time)
  {
    nuturtlebot_msgs::msg::SensorData sd;
    sd.stamp = current_time;
    sd.left_encoder = static_cast<int32_t>(left_encoder_pos_ * encoder_scaling_);
    sd.right_encoder = static_cast<int32_t>(right_encoder_pos_ * encoder_scaling_);
    sensor_pub_->publish(sd);

    sensor_msgs::msg::JointState js;
    js.header.stamp = current_time;
    js.name = {"wheel_left_joint", "wheel_right_joint"};
    js.position = {left_encoder_pos_, right_encoder_pos_};
    joint_pub_->publish(js);

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

    geometry_msgs::msg::TransformStamped scan_tf;
    scan_tf.header.stamp = current_time;
    scan_tf.header.frame_id = "red/base_footprint";
    scan_tf.child_frame_id = "red/base_scan";
    scan_tf.transform.translation.x = 0.0;
    scan_tf.transform.translation.y = 0.0;
    scan_tf.transform.translation.z = 0.0;
    scan_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(scan_tf);
  }

  void publish_seen_markers(const rclcpp::Time & current_time)
  {
    const auto obs_x = get_parameter("obstacles.x").as_double_array();
    const auto obs_y = get_parameter("obstacles.y").as_double_array();
    visualization_msgs::msg::MarkerArray seen_ma;

    for (size_t i = 0; i < obs_x.size(); ++i) {
      double dx = obs_x[i] - current_pose_.translation().x;
      double dy = obs_y[i] - current_pose_.translation().y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist <= laser_max_range_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "red/base_footprint";
        m.header.stamp = current_time;
        m.ns = "seen_markers";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        double th = current_pose_.rotation();
        m.pose.position.x = dx * std::cos(th) + dy * std::sin(th);
        m.pose.position.y = -dx * std::sin(th) + dy * std::cos(th);
        m.pose.position.z = 0.125;
        m.scale.x = 2.0 * obs_r_; m.scale.y = 2.0 * obs_r_; m.scale.z = 0.25;
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(0.1);
        seen_ma.markers.push_back(m);
      } 
    }
    seen_obs_pub_->publish(seen_ma);
  }

  void publish_laser_scan(const rclcpp::Time & current_time)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = current_time;
    scan.header.frame_id = "red/base_scan";
    scan.angle_min = 0.0;
    scan.angle_max = 2.0 * M_PI;
    scan.angle_increment = M_PI / 180.0;
    scan.time_increment = 0.0;
    scan.scan_time = dt_;
    scan.range_min = 0.12;
    scan.range_max = laser_max_range_;
    size_t num_readings = 360;
    scan.ranges.assign(num_readings, laser_max_range_ - 0.01);
    scan.intensities.assign(num_readings, 1.0);
    scan_pub_->publish(scan);
  }

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
      m.scale.x = 2.0 * r; m.scale.y = 2.0 * r; m.scale.z = 0.25;
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    obs_pub_->publish(ma);
  }

  void publish_walls()
  {
    const auto x_len = get_parameter("arena_x_length").as_double();
    const auto y_len = get_parameter("arena_y_length").as_double();
    const double thick = 0.01; const double height = 0.25;
    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "nusim/world";
      m.header.stamp = get_clock()->now();
      m.ns = "walls"; m.id = i;
      m.type = visualization_msgs::msg::Marker::CUBE;
      if (i < 2) { 
        m.pose.position.x = (i == 0) ? x_len / 2.0 + thick / 2.0 : -x_len / 2.0 - thick / 2.0;
        m.scale.x = thick; m.scale.y = y_len + 2.0 * thick;
      } else { 
        m.pose.position.y = (i == 2) ? y_len / 2.0 + thick / 2.0 : -y_len / 2.0 - thick / 2.0;
        m.scale.x = x_len + 2.0 * thick; m.scale.y = thick;
      }
      m.pose.position.z = height / 2.0; m.scale.z = height;
      m.color.r = 1.0; m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    wall_pub_->publish(ma);
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    reset_to_initial_pose();
    publish_walls();
    const auto ox = get_parameter("obstacles.x").as_double_array();
    const auto oy = get_parameter("obstacles.y").as_double_array();
    publish_obstacles(ox, oy, obs_r_);
    RCLCPP_INFO(get_logger(), "Simulation Reset.");
  }

  // Member variables
  uint64_t timestep_;
  double dt_, obs_r_, laser_max_range_;
  double motor_scaling_, encoder_scaling_;
  double left_wheel_pos_, right_wheel_pos_, left_encoder_pos_, right_encoder_pos_;
  double u_l_cmd_, u_r_cmd_, v_l_physics_, v_r_physics_, v_l_sensor_, v_r_sensor_;
  bool draw_only_;
  double input_noise_, slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;

  std::normal_distribution<double> sensor_dist_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub_;

  turtlelib::Transform2D current_pose_;
  turtlelib::DiffDrive diff_robot_{0.0, 0.0};
  nav_msgs::msg::Path path_msg_;

  std::default_random_engine rng_;
  std::normal_distribution<double> gaussian_dist_;
  std::uniform_real_distribution<double> slip_dist_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_, obs_pub_, seen_obs_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_sub_;
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

