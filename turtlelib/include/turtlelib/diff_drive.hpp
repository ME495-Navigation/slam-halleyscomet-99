#ifndef TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD

/// \file
/// \brief Modeling the kinematics of a differential drive robot.

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
/// \brief Represents the position of the robot's wheels
struct WheelPositions
{
  /// \brief Left wheel position in radians
  double left = 0.0;
  /// \brief Right wheel position in radians
  double right = 0.0;
};

/// \brief Represents the velocity of the robot's wheels
struct WheelVelocities
{
  /// \brief Left wheel velocity in rad/s
  double left = 0.0;
  /// \brief Right wheel velocity in rad/s
  double right = 0.0;
};

/// \brief A class to model the kinematics of a differential drive robot
class DiffDrive
{
public:
  /// \brief Default constructor - robot at origin with standard dimensions
  DiffDrive();

  /// \brief Create a DiffDrive model
  /// \param track - the distance between the wheels
  /// \param radius - the radius of the wheels
  DiffDrive(double track, double radius);

  /// \brief Create a DiffDrive model with initial configuration
  /// \param track - the distance between the wheels
  /// \param radius - the radius of the wheels
  /// \param q - the initial configuration
  DiffDrive(double track, double radius, Transform2D q);

  /// \brief Update the robot's configuration based on new wheel positions (Forward Kinematics)
  void forward_kinematics(WheelPositions new_pos);

  /// \brief Compute wheel velocities required for a given body twist (Inverse Kinematics)
  WheelVelocities inverse_kinematics(Twist2D cmd) const;

  /// \brief Get the current configuration of the robot
  Transform2D configuration() const;

  /// \brief Get the current wheel positions
  WheelPositions wheel_positions() const;

  /// \brief Get the track width of the robot
  /// \return the track width in meters
  double track_width() const { return track_width_; }

  /// \brief Get the wheel radius of the robot
  /// \return the wheel radius in meters
  double wheel_radius() const { return wheel_radius_; }

private:
  double track_width_ = 0.16;   ///< Distance between wheels
  double wheel_radius_ = 0.033; ///< Radius of the wheels
  Transform2D q_ = {};          ///< Robot configuration (x, y, theta)
  WheelPositions w_ = {};       ///< Current wheel positions
};
}

#endif
