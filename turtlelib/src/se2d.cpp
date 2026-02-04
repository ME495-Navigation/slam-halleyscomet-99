/// \file
/// \brief Implementation of 2D rigid body transformations and twist integration.

#include <iostream>
#include <cmath>
#include <string>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
std::istream & operator>>(std::istream & is, Twist2D & tw)
{
  auto unit_str = std::string("");
  auto c = '\0';
  if (is.peek() == '<') {
    is >> c >> tw.omega;
    if (std::isalpha(is.peek())) {
      is >> unit_str;
    }
    is >> c >> tw.x >> c >> tw.y >> c;
  } else {
    is >> tw.omega;
    if (std::isalpha(is.peek())) {
      is >> unit_str;
    }
    is >> tw.x >> tw.y;
  }

  if (!unit_str.empty() && unit_str.at(0) == 'd') {
    tw.omega = deg2rad(tw.omega);
  }
  return is;
}

// Twist2D Operators
Twist2D & Twist2D::operator*=(const double s)
{
  omega *= s;
  x *= s;
  y *= s;
  return *this;
}

Twist2D operator*(Twist2D lhs, const double rhs)
{
  return lhs *= rhs;
}

Twist2D operator*(const double lhs, Twist2D rhs)
{
  return rhs *= lhs;
}

// Transform2D Implementation
Transform2D::Transform2D()
: x_(0.0), y_(0.0), theta_(0.0) {}

Transform2D::Transform2D(Vector2D trans)
: x_(trans.x), y_(trans.y), theta_(0.0) {}

Transform2D::Transform2D(double radians)
: x_(0.0), y_(0.0), theta_(radians) {}

Transform2D::Transform2D(Vector2D trans, double radians)
: x_(trans.x), y_(trans.y), theta_(radians) {}

Point2D Transform2D::operator()(Point2D p) const
{
  const auto s = std::sin(theta_);
  const auto c = std::cos(theta_);
  return {p.x * c - p.y * s + x_,
    p.x * s + p.y * c + y_};
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  const auto s = std::sin(theta_);
  const auto c = std::cos(theta_);
  return {v.x * c - v.y * s,
    v.x * s + v.y * c};
}

Twist2D Transform2D::operator()(Twist2D v) const
{
  const auto s = std::sin(theta_);
  const auto c = std::cos(theta_);
  return {v.omega,
    y_ * v.omega + c * v.x - s * v.y,
    -x_ * v.omega + s * v.x + c * v.y};
}

Transform2D Transform2D::inv() const
{
  const auto s = std::sin(theta_);
  const auto c = std::cos(theta_);
  const auto inv_theta = -theta_;
  const auto inv_x = -(x_ * c + y_ * s);
  const auto inv_y = -(-x_ * s + y_ * c);
  return Transform2D({inv_x, inv_y}, inv_theta);
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  const auto s = std::sin(theta_);
  const auto c = std::cos(theta_);
  const auto new_x = rhs.x_ * c - rhs.y_ * s + x_;
  const auto new_y = rhs.x_ * s + rhs.y_ * c + y_;
  theta_ += rhs.theta_;
  x_ = new_x;
  y_ = new_y;
  return *this;
}

Vector2D Transform2D::translation() const
{
  return {x_, y_};
}

double Transform2D::rotation() const
{
  return theta_;
}

// Task B.9: Integrate Twist
Transform2D integrate_twist(Twist2D tw)
{
  if (almost_equal(tw.omega, 0.0)) {
        // Case 1: Pure translation
    return Transform2D({tw.x, tw.y}, 0.0);
  } else {
        // Case 2: Translation and Rotation (Rigid body following a circular arc)
        // Center of rotation in body frame: s = [y/w, -x/w]^T
        // Transform = Tsb * T_pure_rot(w) * Tbs
    const auto s = std::sin(tw.omega);
    const auto c = std::cos(tw.omega);

    const auto dx = (tw.x * s + tw.y * (c - 1.0)) / tw.omega;
    const auto dy = (tw.y * s + tw.x * (1.0 - c)) / tw.omega;

    return Transform2D({dx, dy}, tw.omega);
  }
}

// I/O Operators
std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  auto theta = 0.0;
  auto dx = 0.0;
  auto dy = 0.0;
  auto unit_str = std::string("");
  auto c = '\0';

  if (is.peek() == '{') {
    is >> c;
    is >> theta;
    is >> std::ws;
    if (std::isalpha(is.peek())) {
      is >> unit_str;
    }
    if (is.peek() == ',') {
      is >> c;
    }
    is >> dx;
    if (is.peek() == ',') {
      is >> c;
    }
    is >> dy;
    if (is.peek() == '}') {
      is >> c;
    }
  } else {
    is >> theta;
    is >> std::ws;
    if (std::isalpha(is.peek())) {
      is >> unit_str;
    }
    is >> dx >> dy;
  }

  if (!unit_str.empty() && unit_str.at(0) == 'd') {
    theta = deg2rad(theta);
  }

  tf = Transform2D({dx, dy}, theta);
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  return lhs *= rhs;
}
} // namespace turtlelib
