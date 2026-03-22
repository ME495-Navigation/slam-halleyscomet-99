/// \file
/// \brief EKF SLAM node with unknown data association using Euclidean distance gating.
///
/// The state vector is ξ = [x, y, θ, m1x, m1y, ..., mNx, mNy]ᵀ.
/// The prediction step uses differential-drive forward kinematics applied to
/// wheel angle deltas from red/joint_states.  The measurement update uses
/// range-bearing observations from the landmarks node.  Data association is
/// performed by projecting measurements into the map frame and finding the
/// nearest known landmark within a Euclidean distance gate.
///
/// PARAMETERS:
///     wheel_radius (double): Radius of the robot wheels [m].
///     track_width (double): Distance between wheel centres [m].
///     process_noise (double): Diagonal variance of the robot-pose process noise.
///     sensor_noise (double): Diagonal variance of the range/bearing sensor noise.
///     assoc_threshold (double): Euclidean distance gate for data association [m].
///     max_landmarks (int): Maximum number of landmarks the EKF will track.
/// PUBLISHES:
///     ~/path (nav_msgs::msg::Path): SLAM estimated robot trajectory (green).
///     ~/map_landmarks (visualization_msgs::msg::MarkerArray): SLAM landmark estimates.
/// SUBSCRIBES:
///     red/joint_states (sensor_msgs::msg::JointState): Wheel positions for prediction step.
///     /landmarks/detected_landmarks (visualization_msgs::msg::MarkerArray): Fitted landmark
///         positions in the laser scan frame, used as range-bearing measurements.
/// BROADCASTS:
///     map -> green/base_footprint: SLAM estimated robot pose.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// \brief EKF SLAM node implementing landmark-based localisation with unknown data association.
class EkfSlam : public rclcpp::Node
{
public:
  /// \brief Constructor: declares parameters, initialises EKF state, sets up ROS interfaces.
  EkfSlam()
  : Node("slam"),
    process_noise_(1e-3),
    sensor_noise_(5e-1),
    assoc_threshold_(0.5),
    max_landmarks_(20),
    n_landmarks_(0),
    robot_(0.0, 0.0),
    prev_left_(0.0),
    prev_right_(0.0),
    first_js_(true),
    last_path_time_(0.0),
    xi_(Eigen::VectorXd::Zero(3)),
    Sigma_(Eigen::MatrixXd::Zero(3, 3)),
    xi_odom_(Eigen::Vector3d::Zero())
  {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("process_noise", 1e-3);
    declare_parameter("sensor_noise", 5e-1);
    declare_parameter("assoc_threshold", 0.5);
    declare_parameter("max_landmarks", 20);

    const double wheel_radius = get_parameter("wheel_radius").as_double();
    const double track_width = get_parameter("track_width").as_double();
    process_noise_ = get_parameter("process_noise").as_double();
    sensor_noise_ = get_parameter("sensor_noise").as_double();
    assoc_threshold_ = get_parameter("assoc_threshold").as_double();
    max_landmarks_ = get_parameter("max_landmarks").as_int();

    robot_ = turtlelib::DiffDrive(track_width, wheel_radius);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "red/joint_states", 10,
      std::bind(&EkfSlam::joint_callback, this, std::placeholders::_1));

    landmark_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/landmarks/detected_landmarks", 10,
      std::bind(&EkfSlam::landmark_callback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    map_landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/map_landmarks", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_msg_.header.frame_id = "map";

    RCLCPP_INFO(
      get_logger(),
      "EKF SLAM ready. process_noise=%.2e sensor_noise=%.2e "
      "assoc_threshold=%.2f m max_landmarks=%d",
      process_noise_, sensor_noise_, assoc_threshold_, max_landmarks_);
  }

private:
  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------

  /// \brief EKF prediction step triggered by each wheel joint-state message.
  ///
  /// The TF is broadcast on every callback regardless of the dead-band so that
  /// the green robot stays visible in RViz at all times.  The dead-band only
  /// gates the EKF prediction and path updates: accumulating sub-threshold
  /// encoder noise into the filter would cause spurious drift on the real robot
  /// while the motors are off.
  ///
  /// \param msg Incoming JointState with cumulative wheel angles.
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double left = 0.0, right = 0.0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name.at(i) == "wheel_left_joint") {left = msg->position.at(i);}
      if (msg->name.at(i) == "wheel_right_joint") {right = msg->position.at(i);}
    }

    if (first_js_) {
      prev_left_ = left;
      prev_right_ = right;
      first_js_ = false;
      // Broadcast the initial (zero) pose immediately so the green robot
      // appears in RViz as soon as the node starts.
      broadcast_transform(msg->header.stamp);
      return;
    }

    const double dl = left - prev_left_;
    const double dr = right - prev_right_;

    // Always broadcast TF so the green robot remains visible even when the
    // dead-band suppresses prediction.  TF must be published at the message
    // timestamp (not wall clock) to stay consistent with robot_state_publisher.
    broadcast_transform(msg->header.stamp);

    // ############################ Begin_Citation [5] ############################
    // Dead-band: only run the EKF prediction when the delta is large enough
    // to represent real motion.  Physical encoders produce non-zero noise even
    // when stationary; feeding these into the filter causes spurious drift.
    constexpr double ENCODER_DEADBAND = 0.01;
    if (std::abs(dl) < ENCODER_DEADBAND && std::abs(dr) < ENCODER_DEADBAND) {
      return;
    }
    // ############################ End_Citation [5] ############################

    prev_left_ = left;
    prev_right_ = right;

    ekf_predict(dl, dr);
    update_path(msg->header.stamp);
  }

  /// \brief EKF measurement update triggered by each batch of detected landmarks.
  /// \param msg Incoming MarkerArray of fitted landmark cylinder positions in the laser frame.
  void landmark_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (first_js_) {return;}
    if (msg->markers.empty()) {return;}

    const rclcpp::Time stamp = msg->markers.at(0).header.stamp;

    for (const auto & marker : msg->markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD) {continue;}

      const double lx = marker.pose.position.x;
      const double ly = marker.pose.position.y;
      const double r_m = std::sqrt(lx * lx + ly * ly);
      const double phi_m = std::atan2(ly, lx);

      // Discard measurements outside the sensor's valid range.
      if (r_m < 0.10 || r_m > 3.0) {continue;}

      const int j = data_associate(r_m, phi_m);

      if (j < 0) {
        // Only initialise new landmarks up to max_landmarks_ to prevent
        // runaway state growth from real-world false positives.
        if (n_landmarks_ < max_landmarks_) {
          add_landmark(r_m, phi_m);
          ekf_update(r_m, phi_m, n_landmarks_ - 1);
        }
      } else {
        ekf_update(r_m, phi_m, j);
      }
    }

    broadcast_transform(stamp);
    publish_map_landmarks(stamp);
  }

  // ---------------------------------------------------------------------------
  // EKF core
  // ---------------------------------------------------------------------------

  /// \brief EKF prediction step: propagates robot pose and covariance.
  ///
  /// Body-frame displacement (dx_b, dy_b, dtheta) is computed from wheel deltas
  /// via a temporary zero-origin DiffDrive.  World-frame displacement is obtained
  /// by rotating by the current heading θ.
  ///
  /// \param dl Change in left wheel angle [rad].
  /// \param dr Change in right wheel angle [rad].
  void ekf_predict(double dl, double dr)
  {
    turtlelib::DiffDrive temp(robot_.track_width(), robot_.wheel_radius());
    temp.forward_kinematics({dl, dr});
    const double dx_b = temp.configuration().translation().x;
    const double dy_b = temp.configuration().translation().y;
    const double dtheta = temp.configuration().rotation();

    const double theta = xi_(2);
    const int    sz = 3 + 2 * n_landmarks_;

    const double dx_w = dx_b * std::cos(theta) - dy_b * std::sin(theta);
    const double dy_w = dx_b * std::sin(theta) + dy_b * std::cos(theta);

    const double g13 = -dx_b * std::sin(theta) - dy_b * std::cos(theta);
    const double g23 = dx_b * std::cos(theta) - dy_b * std::sin(theta);

    xi_(0) += dx_w;
    xi_(1) += dy_w;
    xi_(2) = normalize_angle(xi_(2) + dtheta);

    const double theta_o = xi_odom_(2);
    xi_odom_(0) += dx_b * std::cos(theta_o) - dy_b * std::sin(theta_o);
    xi_odom_(1) += dx_b * std::sin(theta_o) + dy_b * std::cos(theta_o);
    xi_odom_(2) = normalize_angle(xi_odom_(2) + dtheta);

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(sz, sz);
    F(0, 2) = g13;
    F(1, 2) = g23;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(sz, sz);
    Q(0, 0) = process_noise_;
    Q(1, 1) = process_noise_;
    Q(2, 2) = process_noise_;

    Sigma_ = F * Sigma_ * F.transpose() + Q;
  }

  /// \brief Data association via Euclidean distance gating in the map frame.
  ///
  /// Projects the range-bearing measurement into the map frame and returns the
  /// nearest known landmark within assoc_threshold_, or −1 for a new landmark.
  ///
  /// \param r_m   Measured range [m].
  /// \param phi_m Measured bearing [rad].
  /// \return Associated landmark index, or −1 for a new landmark.
  // ############################ Begin_Citation [4] ############################
  // Euclidean distance gating in the map frame for data association.
  // Mahalanobis distance was not used because newly initialised landmarks
  // carry covariance ~1e6, collapsing Mahalanobis distance to near zero
  // and causing systematic false associations.
  int data_associate(double r_m, double phi_m) const
  {
    const double mx_pred = xi_(0) + r_m * std::cos(phi_m + xi_(2));
    const double my_pred = xi_(1) + r_m * std::sin(phi_m + xi_(2));

    double min_dist = assoc_threshold_;
    int    best_j = -1;

    for (int j = 0; j < n_landmarks_; ++j) {
      const double ddx = mx_pred - xi_(3 + 2 * j);
      const double ddy = my_pred - xi_(3 + 2 * j + 1);
      const double dist = std::sqrt(ddx * ddx + ddy * ddy);

      if (dist < min_dist) {
        min_dist = dist;
        best_j = j;
      }
    }

    return best_j;
  }
  // ############################ End_Citation [4] ############################

  /// \brief Extends the state vector and covariance to include a new landmark.
  ///
  /// \param r_m   Measured range [m].
  /// \param phi_m Measured bearing [rad].
  void add_landmark(double r_m, double phi_m)
  {
    const double theta = xi_(2);
    const double mx = xi_(0) + r_m * std::cos(phi_m + theta);
    const double my = xi_(1) + r_m * std::sin(phi_m + theta);

    const int old_sz = 3 + 2 * n_landmarks_;
    const int new_sz = old_sz + 2;

    Eigen::VectorXd xi_new = Eigen::VectorXd::Zero(new_sz);
    xi_new.head(old_sz) = xi_;
    xi_new(old_sz) = mx;
    xi_new(old_sz + 1) = my;
    xi_ = xi_new;

    constexpr double INIT_LM_COV = 1e6;
    Eigen::MatrixXd Sigma_new = Eigen::MatrixXd::Zero(new_sz, new_sz);
    Sigma_new.topLeftCorner(old_sz, old_sz) = Sigma_;
    Sigma_new(old_sz, old_sz) = INIT_LM_COV;
    Sigma_new(old_sz + 1, old_sz + 1) = INIT_LM_COV;
    Sigma_ = Sigma_new;

    n_landmarks_++;
    RCLCPP_DEBUG(get_logger(), "New landmark %d at (%.2f, %.2f)", n_landmarks_, mx, my);
  }

  /// \brief EKF measurement update for an associated landmark.
  ///
  /// \param r_m   Measured range [m].
  /// \param phi_m Measured bearing [rad].
  /// \param j     Index of the associated landmark.
  void ekf_update(double r_m, double phi_m, int j)
  {
    const int sz = 3 + 2 * n_landmarks_;

    const double delta_x = xi_(3 + 2 * j) - xi_(0);
    const double delta_y = xi_(3 + 2 * j + 1) - xi_(1);
    const double d2 = delta_x * delta_x + delta_y * delta_y;
    const double d = std::sqrt(d2);

    if (d < 1e-6) {return;}

    const double phi_hat = normalize_angle(std::atan2(delta_y, delta_x) - xi_(2));

    Eigen::Vector2d nu;
    nu(0) = r_m - d;
    nu(1) = normalize_angle(phi_m - phi_hat);

    const Eigen::MatrixXd H = build_H(j, delta_x, delta_y, d, d2, sz);
    const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sensor_noise_;
    const Eigen::Matrix2d S = (H * Sigma_ * H.transpose()).eval() + R;
    const Eigen::MatrixXd K = Sigma_ * H.transpose() * S.inverse();

    xi_ += K * nu;
    xi_(2) = normalize_angle(xi_(2));

    Sigma_ = (Eigen::MatrixXd::Identity(sz, sz) - K * H) * Sigma_;
  }

  /// \brief Builds the 2 × sz range-bearing measurement Jacobian for landmark j.
  ///
  /// \param j   Landmark index.
  /// \param dx  Map-frame x-distance from robot to landmark.
  /// \param dy  Map-frame y-distance from robot to landmark.
  /// \param d   Euclidean distance [m].
  /// \param d2  Squared distance [m²].
  /// \param sz  Total state size.
  /// \return 2 × sz Jacobian H.
  Eigen::MatrixXd build_H(
    int j, double dx, double dy, double d, double d2, int sz) const
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, sz);

    H(0, 0) = -dx / d;
    H(0, 1) = -dy / d;
    H(0, 2) = 0.0;
    H(1, 0) = dy / d2;
    H(1, 1) = -dx / d2;
    H(1, 2) = -1.0;

    H(0, 3 + 2 * j) = dx / d;
    H(0, 3 + 2 * j + 1) = dy / d;
    H(1, 3 + 2 * j) = -dy / d2;
    H(1, 3 + 2 * j + 1) = dx / d2;

    return H;
  }

  // ---------------------------------------------------------------------------
  // Publishing helpers
  // ---------------------------------------------------------------------------

  /// \brief Broadcasts map → green/base_footprint with the current SLAM estimate.
  ///
  /// Called on every joint_states message (100 Hz) so the green robot is always
  /// visible.  Also called from landmark_callback when the state is updated.
  ///
  /// \param stamp Timestamp for the TF message.
  void broadcast_transform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "green/base_footprint";
    t.transform.translation.x = xi_(0);
    t.transform.translation.y = xi_(1);
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, xi_(2));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  /// \brief Appends the current SLAM pose to the path and publishes it.
  ///
  /// Throttled to ~5 Hz (PATH_PERIOD = 0.2 s) to avoid flooding the path
  /// message with micro-corrections from the 100 Hz encoder callback.
  ///
  /// \param stamp Timestamp for the PoseStamped entry.
  void update_path(const rclcpp::Time & stamp)
  {
    // ############################ Begin_Citation [6] ############################
    // Throttle path updates to ~5 Hz to avoid a dense jagged path from the
    // 100 Hz encoder callback on the real robot.
    constexpr double PATH_PERIOD = 0.2;
    const double t = stamp.seconds();
    if (t - last_path_time_ < PATH_PERIOD) {return;}
    last_path_time_ = t;
    // ############################ End_Citation [6] ############################

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = xi_(0);
    ps.pose.position.y = xi_(1);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, xi_(2));
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    path_msg_.poses.push_back(ps);
    path_msg_.header.stamp = stamp;
    path_pub_->publish(path_msg_);
  }

  /// \brief Publishes green cylinder markers for all SLAM-estimated landmarks.
  /// \param stamp Timestamp for the marker headers.
  void publish_map_landmarks(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray ma;
    for (int j = 0; j < n_landmarks_; ++j) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = "map_landmarks";
      m.id = j;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = xi_(3 + 2 * j);
      m.pose.position.y = xi_(3 + 2 * j + 1);
      m.pose.position.z = 0.125;
      constexpr double PILLAR_DIAMETER = 0.076;
      m.scale.x = PILLAR_DIAMETER;
      m.scale.y = PILLAR_DIAMETER;
      m.scale.z = 0.25;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      ma.markers.push_back(m);
    }
    map_landmark_pub_->publish(ma);
  }

  // ---------------------------------------------------------------------------
  // Utility
  // ---------------------------------------------------------------------------

  /// \brief Wraps an angle to the range [−π, π].
  /// \param a Input angle [rad].
  /// \return Normalised angle [rad].
  static double normalize_angle(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  /// \brief Variance of the robot-pose process noise (diagonal, per prediction step).
  double process_noise_;

  /// \brief Variance of range and bearing sensor noise (diagonal).
  double sensor_noise_;

  /// \brief Euclidean distance gate threshold for data association [m].
  double assoc_threshold_;

  /// \brief Maximum number of landmarks the EKF will initialise.
  int max_landmarks_;

  /// \brief Number of landmarks currently tracked in the EKF state.
  int n_landmarks_;

  /// \brief Differential-drive model storing track_width and wheel_radius.
  turtlelib::DiffDrive robot_;

  /// \brief Previous cumulative wheel angles [rad], used to compute per-step deltas.
  double prev_left_, prev_right_;

  /// \brief True until the first joint-state message is received.
  bool first_js_;

  /// \brief Wall-clock time [s] of the last path pose appended.
  double last_path_time_;

  /// \brief Full EKF state vector ξ = [x, y, θ, m1x, m1y, …]ᵀ.
  Eigen::VectorXd xi_;

  /// \brief Full EKF covariance Σ, size (3 + 2N) × (3 + 2N).
  Eigen::MatrixXd Sigma_;

  /// \brief Pure odometry estimate [x, y, θ]ᵀ, no EKF correction applied.
  ///        Used to report final odometry error in README.md.
  Eigen::Vector3d xi_odom_;

  /// \brief Accumulated SLAM path published on ~/path.
  nav_msgs::msg::Path path_msg_;

  /// \brief Subscription to wheel joint states (prediction step trigger).
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  /// \brief Subscription to detected landmark positions (measurement update trigger).
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_sub_;

  /// \brief Publisher for the SLAM estimated path.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /// \brief Publisher for SLAM estimated landmark cylinders.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_landmark_pub_;

  /// \brief TF broadcaster for map → green/base_footprint.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfSlam>());
  rclcpp::shutdown();
  return 0;
}
