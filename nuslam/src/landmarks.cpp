/// \file
/// \brief Landmark detection node that identifies cylindrical pillars from LIDAR scans.
///
/// PARAMETERS:
///     threshold (double): Distance threshold for clustering adjacent LIDAR points [m].
///     laser_height (double): Height of the laser scan frame above the ground plane [m].
///         Used to compensate marker z so detected cylinders rest on the ground (z = 0).
///         Default 0.172 m matches the TurtleBot3 Burger LiDAR mount height.
///     marker_lifetime (double): Duration in seconds that each published marker remains
///         visible in RViz before expiring.  Set longer than the scan period to avoid
///         flickering on the real robot (default 4.0 s for ~2.5 Hz LiDAR).
/// PUBLISHES:
///     ~/detected_landmarks (visualization_msgs::msg::MarkerArray): Magenta cylinders at
///         fitted landmark locations expressed in the laser scan frame.
///     ~/clusters (visualization_msgs::msg::MarkerArray): LIDAR points coloured by cluster
///         ID, used to debug the clustering step in RViz.
/// SUBSCRIBES:
///     /nusimulator/laser_scan (sensor_msgs::msg::LaserScan): Simulated or real LIDAR data.
///         When running on the real robot this topic is remapped to /scan in the launch file.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;

/// \brief Performs unsupervised clustering and supervised circular regression on LIDAR data
///        to detect cylindrical landmarks (pillars) and publish their estimated positions.
class Landmarks : public rclcpp::Node
{
public:
  /// \brief Initialises ROS parameters, the laser scan subscription, and marker publishers.
  Landmarks()
  : Node("landmarks")
  {
    declare_parameter("threshold", 0.15);
    declare_parameter("laser_height", 0.172);
    declare_parameter("marker_lifetime", 4.0);

    dist_threshold_ = get_parameter("threshold").as_double();
    laser_height_ = get_parameter("laser_height").as_double();
    marker_lifetime_ = get_parameter("marker_lifetime").as_double();

    // ############################ Begin_Citation [2] ############################
    // Use BEST_EFFORT QoS to match the real TurtleBot3 LiDAR driver, which publishes
    // with BEST_EFFORT reliability.  A RELIABLE subscriber would be incompatible and
    // receive no messages from the physical sensor.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    // ############################ End_Citation [2] ############################
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/nusimulator/laser_scan", qos,
      std::bind(&Landmarks::scan_callback, this, std::placeholders::_1));

    landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/detected_landmarks", 10);
    cluster_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/clusters", 10);

    RCLCPP_INFO(
      get_logger(),
      "Landmark node ready. threshold=%.3f m  laser_height=%.3f m  marker_lifetime=%.1f s",
      dist_threshold_, laser_height_, marker_lifetime_);
  }

private:
  // ---------------------------------------------------------------------------
  // LaserScan callback
  // ---------------------------------------------------------------------------

  /// \brief Processes each incoming LaserScan: converts ranges to Cartesian coordinates,
  ///        clusters nearby points, classifies circular arcs, fits circles with the Pratt
  ///        method, and publishes landmark cylinder markers.
  /// \param msg Shared pointer to the incoming LaserScan message.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    dist_threshold_ = get_parameter("threshold").as_double();
    laser_height_ = get_parameter("laser_height").as_double();
    marker_lifetime_ = get_parameter("marker_lifetime").as_double();

    // -------------------------------------------------------------------------
    // A. Polar to Cartesian conversion.
    // -------------------------------------------------------------------------
    std::vector<turtlelib::Point2D> points;
    points.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = static_cast<double>(msg->ranges.at(i));
      if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
        const double theta =
          msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        points.push_back({r * std::cos(theta), r * std::sin(theta)});
      }
    }

    if (points.empty()) {return;}

    // -------------------------------------------------------------------------
    // B. Distance-based clustering.
    // -------------------------------------------------------------------------
    constexpr size_t MIN_CLUSTER_SIZE = 3;

    std::vector<std::vector<turtlelib::Point2D>> clusters;
    std::vector<turtlelib::Point2D> current_cluster;
    current_cluster.push_back(points.at(0));

    for (size_t i = 1; i < points.size(); ++i) {
      const double d = turtlelib::distance(points.at(i), points.at(i - 1));
      if (d < dist_threshold_) {
        current_cluster.push_back(points.at(i));
      } else {
        if (current_cluster.size() >= MIN_CLUSTER_SIZE) {
          clusters.push_back(current_cluster);
        }
        current_cluster.clear();
        current_cluster.push_back(points.at(i));
      }
    }
    if (current_cluster.size() >= MIN_CLUSTER_SIZE) {
      clusters.push_back(current_cluster);
    }

    // -------------------------------------------------------------------------
    // C. Wrap-around merge for 360° scanners.
    // -------------------------------------------------------------------------
    if (clusters.size() > 1) {
      const double d_wrap =
        turtlelib::distance(clusters.back().back(), clusters.front().front());
      if (d_wrap < dist_threshold_) {
        clusters.front().insert(
          clusters.front().begin(),
          clusters.back().begin(),
          clusters.back().end());
        clusters.pop_back();
      }
    }

    // -------------------------------------------------------------------------
    // D. Classification and Pratt circle fitting.
    // -------------------------------------------------------------------------
    visualization_msgs::msg::MarkerArray landmark_ma;
    int id = 0;

    for (const auto & cluster : clusters) {
      if (is_circle(cluster)) {
        try {
          const auto circle = turtlelib::fit_circle(cluster);

          if (circle.radius > 0.01 && circle.radius < 0.20) {
            landmark_ma.markers.push_back(
              create_landmark_marker(circle, id++,
                msg->header.frame_id, msg->header.stamp));
          }
        } catch (const std::exception & e) {
          RCLCPP_DEBUG(get_logger(), "Circle fit error: %s", e.what());
        }
      }
    }

    landmark_pub_->publish(landmark_ma);
    publish_cluster_markers(clusters, msg->header.frame_id, msg->header.stamp);
  }

  // ---------------------------------------------------------------------------
  // Classification
  // ---------------------------------------------------------------------------

  /// \brief Classifies a cluster as a circular arc using the Inscribed Angle Theorem.
  ///
  /// Reference: J. Xavier et al., "Fast line, arc/circle and leg detection from laser
  /// scan data in a Player driver," ICRA 2005.
  ///
  /// Thresholds are relaxed relative to the simulation values to account for the
  /// higher sensor noise of the physical LiDAR:
  ///   - stdev < 0.35 rad  (simulation: 0.25)
  ///   - mean  60°–170°    (simulation: 80°–170°)
  ///
  /// \param cluster Vector of 2D points belonging to a single candidate cluster.
  /// \return True if the cluster is geometrically consistent with a circular arc.
  bool is_circle(const std::vector<turtlelib::Point2D> & cluster)
  {
    if (cluster.size() < 4) {return false;}

    const auto p1 = cluster.front();
    const auto p2 = cluster.back();

    std::vector<double> angles;
    angles.reserve(cluster.size() - 2);

    for (size_t i = 1; i < cluster.size() - 1; ++i) {
      const auto p = cluster.at(i);
      const double a = std::atan2(p1.y - p.y, p1.x - p.x);
      const double b = std::atan2(p2.y - p.y, p2.x - p.x);
      double diff = a - b;
      while (diff > M_PI) {diff -= 2.0 * M_PI;}
      while (diff < -M_PI) {diff += 2.0 * M_PI;}
      angles.push_back(std::abs(diff));
    }

    const double n = static_cast<double>(angles.size());

    double sum = 0.0;
    for (const double a : angles) {sum += a;}
    const double mean = sum / n;
    const double mean_deg = mean * (180.0 / M_PI);

    double sq_sum = 0.0;
    for (const double a : angles) {sq_sum += (a - mean) * (a - mean);}
    const double stdev = std::sqrt(sq_sum / n);

    return  stdev < 0.35 && mean_deg > 60.0 && mean_deg < 170.0;
  }

  // ---------------------------------------------------------------------------
  // Marker helpers
  // ---------------------------------------------------------------------------

  /// \brief Builds a magenta cylinder marker centred at the fitted circle centre.
  ///
  /// The marker z compensates for the laser scan frame height above the ground:
  ///   world_z = (0.125 - laser_height_) + laser_height_ = 0.125 m
  /// so the cylinder base rests on z = 0.  The diameter is fixed to the known
  /// pillar size (0.076 m) because Pratt fitting on a partial arc overestimates
  /// the true radius.  Lifetime is marker_lifetime_ to prevent flickering.
  ///
  /// \param c     Fitted circle whose centre determines the marker XY position.
  /// \param id    Unique integer marker ID within this array.
  /// \param frame TF frame ID copied from the LaserScan header.
  /// \param stamp Timestamp copied from the LaserScan header.
  /// \return A fully configured Marker message ready to publish.
  visualization_msgs::msg::Marker create_landmark_marker(
    const turtlelib::Circle & c,
    int id,
    const std::string & frame,
    rclcpp::Time stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = "detected_landmarks";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = c.center.x;
    m.pose.position.y = c.center.y;
    m.pose.position.z = 0.125 - laser_height_;

    constexpr double pillar_diameter = 0.076;
    m.scale.x = pillar_diameter;
    m.scale.y = pillar_diameter;
    m.scale.z = 0.25;

    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 1.0;

    m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);

    return m;
  }

  /// \brief Publishes small coloured sphere markers for every point in every cluster.
  ///
  /// A DELETEALL marker with id=-1 is sent first each scan cycle to clear all stale
  /// markers from the previous cycle.  The id is set to -1 (not the default 0) to
  /// avoid a "Duplicate Marker" warning: both DELETEALL and the first point marker
  /// would otherwise share ns="clusters" and id=0 in the same MarkerArray message.
  ///
  /// Sphere diameter is 0.01 m (1 cm) for a clean, fine-grained visualisation.
  /// Clusters cycle through red / green / blue so adjacent clusters are visually
  /// distinct.  Lifetime matches marker_lifetime_ to prevent flickering.
  ///
  /// \param clusters Each inner vector holds the 2D scan points of one cluster.
  /// \param frame    TF frame ID for the marker header.
  /// \param stamp    Timestamp for the marker header.
  void publish_cluster_markers(
    const std::vector<std::vector<turtlelib::Point2D>> & clusters,
    const std::string & frame,
    rclcpp::Time stamp)
  {
    visualization_msgs::msg::MarkerArray ma;

    // ############################ Begin_Citation [3] ############################
    // Send DELETEALL first to clear stale markers from the previous scan cycle.
    // id=-1 avoids a duplicate-marker warning with the first point marker (id=0)
    // that would otherwise occur because both share the same namespace.
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame;
    delete_all.header.stamp = stamp;
    delete_all.ns = "clusters";
    delete_all.id = -1;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(delete_all);
    // ############################ End_Citation [3] ############################

    int point_id = 0;

    for (size_t i = 0; i < clusters.size(); ++i) {
      for (const auto & p : clusters.at(i)) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = stamp;
        m.ns = "clusters";
        m.id = point_id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        // Small spheres (1 cm diameter) give a fine-grained view of the
        // cluster point distribution without obscuring nearby structure.
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = 0.01;
        m.color.r = (i % 3 == 0) ? 1.0 : 0.0;
        m.color.g = (i % 3 == 1) ? 1.0 : 0.0;
        m.color.b = (i % 3 == 2) ? 1.0 : 0.0;
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
        ma.markers.push_back(m);
      }
    }

    cluster_pub_->publish(ma);
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  /// \brief Euclidean distance threshold for merging consecutive scan points [m].
  double dist_threshold_;

  /// \brief Height of the laser scan frame above the ground plane [m].
  double laser_height_;

  /// \brief Lifetime of published markers [s].
  double marker_lifetime_;

  /// \brief Subscription to the raw LIDAR scan data.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  /// \brief Publisher for the detected landmark cylinder markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;

  /// \brief Publisher for per-cluster debug sphere markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
