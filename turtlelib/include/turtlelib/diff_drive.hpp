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
/// \note This class tracks the configuration q = (x, y, theta) and wheel positions
class DiffDrive
{
public:
  /// \brief Default constructor - robot at origin with standard dimensions
  /// \details Track width: 0.16m, Wheel radius: 0.033m
  DiffDrive();

  /// \brief Create a DiffDrive model
  /// \param track - the distance between the wheels
  /// \param radius - the radius of the wheels
  DiffDrive(double track, double radius);

  /// \brief Create a DiffDrive model with initial configuration
  /// \param track - the distance between the wheels
  /// \param radius - the radius of the wheels
  /// \param q - the initial configuration (x, y, theta)
  DiffDrive(double track, double radius, Transform2D q);

  /// \brief Update the robot's configuration based on new wheel positions (Forward Kinematics)
  /// \param new_pos - the new positions of the wheels in radians
  /// \note Assumes wheels rotate at constant velocity and do not slip
  /// \details See Equation (2) in doc/Kinematics.pdf for the derivation
  void forward_kinematics(WheelPositions new_pos);

  /// \brief Compute the wheel velocities required to achieve a given body twist (Inverse Kinematics)
  /// \param cmd - the desired body twist
  /// \return the required wheel velocities in rad/s
  /// \throw std::logic_error if the twist requires slipping (non-zero y velocity)
  /// \details See Equation (1) in doc/Kinematics.pdf for the derivation
  WheelVelocities inverse_kinematics(Twist2D cmd) const;

  /// \brief Get the current configuration of the robot
  /// \return the robot's pose (Transform2D) relative to the world frame
  Transform2D configuration() const;

  /// \brief Get the current wheel positions
  /// \return the radians each wheel has turned
  WheelPositions wheel_positions() const;

private:
  /// \brief Distance between the center of the wheels
  double track_width_ = 0.16;
  /// \brief Radius of the wheels
  double wheel_radius_ = 0.033;
  /// \brief Current robot pose (x, y, theta)
  Transform2D q_ = {};
  /// \brief Current wheel positions (phi_l, phi_r)
  WheelPositions w_ = {};
};
}

#endif
