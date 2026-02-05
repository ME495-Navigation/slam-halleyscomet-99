/// \file
/// \brief Differential drive kinematics implementation.

#include <stdexcept>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive() {}

DiffDrive::DiffDrive(double track, double radius)
: track_width_(track), wheel_radius_(radius) {}

DiffDrive::DiffDrive(double track, double radius, Transform2D q)
: track_width_(track), wheel_radius_(radius), q_(q) {}

void DiffDrive::forward_kinematics(WheelPositions new_pos)
{
        // 1. Calculate the change in wheel positions
  const auto d_left = new_pos.left - w_.left;
  const auto d_right = new_pos.right - w_.right;

        // 2. Calculate the body twist components for one time unit
        // Delta theta = (r/D) * (d_right - d_left)
        // Delta x = (r/2) * (d_right + d_left)
  const auto d_theta = (wheel_radius_ / track_width_) * (d_right - d_left);
  const auto d_x = (wheel_radius_ / 2.0) * (d_right + d_left);

        // 3. Integrate the twist to get the relative transformation
  const auto delta_tf = integrate_twist({d_theta, d_x, 0.0});

        // 4. Update configuration: q_new = q_old * delta_tf
  q_ *= delta_tf;

        // 5. Update stored wheel positions
  w_ = new_pos;
}

WheelVelocities DiffDrive::inverse_kinematics(Twist2D cmd) const
{
  if (!almost_equal(cmd.y, 0.0)) {
    throw std::logic_error("Twist requires slipping which is not possible for diff drive");
  }

        // dot_phi_l = (1/r) * (v_x - (D/2)*omega)
        // dot_phi_r = (1/r) * (v_x + (D/2)*omega)
  const auto left_vel = (1.0 / wheel_radius_) * (cmd.x - (track_width_ / 2.0) * cmd.omega);
  const auto right_vel = (1.0 / wheel_radius_) * (cmd.x + (track_width_ / 2.0) * cmd.omega);

  return {left_vel, right_vel};
}

Transform2D DiffDrive::configuration() const
{
  return q_;
}

WheelPositions DiffDrive::wheel_positions() const
{
  return w_;
}
}
