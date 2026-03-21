/// \file
/// \brief Landmark detection node that clusters laser scans and fits circles to detected objects.
///
/// \author Halley (Chenwan Zhong)
/// \date March 2026
///
/// PARAMETERS:
///     threshold (double): Distance threshold for clustering laser points [m].
/// PUBLISHES:
///     ~/detected_landmarks (visualization_msgs::msg::MarkerArray): Fitted landmark cylinders.
///     ~/clusters (visualization_msgs::msg::MarkerArray): Colored points showing individual clusters.
/// SUBSCRIBES:
///     /laser_scan (sensor_msgs::msg::LaserScan): Raw 2D LIDAR data.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;

/// \brief Detects circular landmarks from 2D LIDAR scans using clustering and Pratt fitting.
class Landmarks : public rclcpp::Node
{
public:
  /// \brief Constructor for the Landmarks node.
  Landmarks()
  : Node("landmarks")
  {
    // 1. Parameters
    declare_parameter("threshold", 0.15);
    dist_threshold_ = get_parameter("threshold").as_double();

    // 2. Subscriptions
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser_scan", 10, std::bind(&Landmarks::scan_callback, this, std::placeholders::_1));

    // 3. Publishers
    landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/detected_landmarks", 10);
    cluster_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/clusters", 10);

    RCLCPP_INFO(get_logger(), "Landmark detection node started.");
  }

private:
  /// \brief Callback for LIDAR scans to perform clustering and circle fitting.
  /// \param msg - The incoming laser scan message.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // A. Convert Scan to Cartesian Points (LIDAR Frame)
    std::vector<turtlelib::Point2D> points;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (r > msg->range_min && r < msg->range_max) {
        const double angle = msg->angle_min + i * msg->angle_increment;
        points.push_back({r * std::cos(angle), r * std::sin(angle)});
      }
    }

    if (points.empty()) {return;}

    // B. Clustering based on distance threshold
    std::vector<std::vector<turtlelib::Point2D>> clusters;
    if (!points.empty()) {
      std::vector<turtlelib::Point2D> current_cluster;
      current_cluster.push_back(points[0]);

      for (size_t i = 1; i < points.size(); ++i) {
        if (turtlelib::distance(points[i], points[i - 1]) < dist_threshold_) {
          current_cluster.push_back(points[i]);
        } else {
          clusters.push_back(current_cluster);
          current_cluster.clear();
          current_cluster.push_back(points[i]);
        }
      }
      clusters.push_back(current_cluster);
    }

    // Handle Wrap-around (first and last cluster might be the same object)
    if (clusters.size() > 1) {
      if (turtlelib::distance(clusters.front().front(), clusters.back().back()) < dist_threshold_) {
        clusters.front().insert(
          clusters.front().begin(), clusters.back().begin(),
          clusters.back().end());
        clusters.pop_back();
      }
    }

    // C. Circle Fitting and Classification
    visualization_msgs::msg::MarkerArray landmark_ma;
    int id = 0;

    for (const auto & cluster : clusters) {
      // Step 1: Discard small clusters
      if (cluster.size() < 3) {continue;}

      // Step 2: Classification using Inscribed Angle Theorem
      if (is_circle(cluster)) {
        try {
          // Step 3: Circular Regression (Pratt Method)
          turtlelib::Circle circle = turtlelib::fit_circle(cluster);

          // Step 4: Filter by known landmark size (optional but practical)
          if (circle.radius > 0.02 && circle.radius < 0.15) {
            landmark_ma.markers.push_back(
              create_landmark_marker(circle, id++, msg->header.frame_id, msg->header.stamp));
          }
        } catch (const std::invalid_argument & e) {
          continue;
        }
      }
    }
    landmark_pub_->publish(landmark_ma);
    publish_cluster_markers(clusters, msg->header.frame_id, msg->header.stamp);
  }

  /// \brief Validates if a cluster is likely a circle using the standard deviation of angles.
  /// \param cluster - A group of Cartesian points.
  /// \return True if the points form a circular arc.
  bool is_circle(const std::vector<turtlelib::Point2D> & cluster)
  {
    const auto p1 = cluster.front();
    const auto p2 = cluster.back();
    std::vector<double> angles;

    for (size_t i = 1; i < cluster.size() - 1; ++i) {
      const auto p = cluster[i];
      // Vectors from p to endpoints
      const turtlelib::Vector2D v1 = p1 - p;
      const turtlelib::Vector2D v2 = p2 - p;
      angles.push_back(turtlelib::angle(v1, v2));
    }

    // Compute mean and standard deviation
    double sum = 0.0;
    for (double a : angles) {sum += a;}
    double mean = sum / angles.size();

    double sq_sum = 0.0;
    for (double a : angles) {sq_sum += (a - mean) * (a - mean);}
    double stdev = std::sqrt(sq_sum / angles.size());

    // Thresholds from Xavier et. al. (can be tuned)
    return  stdev < 0.15;
  }

  /// \brief Creates a cylinder marker for RViz.
  visualization_msgs::msg::Marker create_landmark_marker(
    const turtlelib::Circle & c, int id, std::string frame, rclcpp::Time stamp)
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
    m.pose.position.z = 0.125;
    m.scale.x = c.radius * 2.0;
    m.scale.y = c.radius * 2.0;
    m.scale.z = 0.25;
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0; // Magenta
    return m;
  }

  /// \brief Publishes small spheres for every LIDAR point, colored by cluster ID.
  void publish_cluster_markers(
    const std::vector<std::vector<turtlelib::Point2D>> & clusters,
    std::string frame, rclcpp::Time stamp)
  {
    visualization_msgs::msg::MarkerArray ma;
    int point_id = 0;
    for (size_t i = 0; i < clusters.size(); ++i) {
      float r = static_cast<float>(i % 3) / 2.0f;
      float g = static_cast<float>((i + 1) % 3) / 2.0f;
      float b = static_cast<float>((i + 2) % 3) / 2.0f;

      for (const auto & p : clusters[i]) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame; m.header.stamp = stamp;
        m.ns = "clusters"; m.id = point_id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = 0.02; m.scale.y = 0.02; m.scale.z = 0.02;
        m.pose.position.x = p.x; m.pose.position.y = p.y;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
        ma.markers.push_back(m);
      }
    }
    cluster_pub_->publish(ma);
  }

  double dist_threshold_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
