/// \file
/// \brief Implementation of 2D geometry primitives including points and vectors.
///
/// PARAMETERS:
///     None
/// PUBLISHES:
///     None
/// SUBSCRIBES:
///     None
/// SERVERS:
///     None
/// CLIENTS:
///     None

#include <iostream>
#include <cmath>
#include <stdexcept>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{
std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  return os << "(" << p.x << " " << p.y << ")";
}

std::istream & operator>>(std::istream & is, Point2D & p)
{
  auto c = is.peek();
  if (c == '(') {
    is.get();                 // consume '('
    is >> p.x;
    is >> std::ws;
    if (is.peek() == ',') {
      is.get();                     // consume ','
    }
    is >> p.y;
    is >> std::ws;
    if (is.peek() == ')') {
      is.get();                     // consume ')'
    }
  } else {
    is >> p.x >> p.y;
  }
  return is;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  return os << "[" << v.x << " " << v.y << "]";
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  auto c = is.peek();
  if (c == '[') {
    is.get();                 // consume '['
    is >> v.x;
    is >> std::ws;
    if (is.peek() == ',') {
      is.get();                     // consume ','
    }
    is >> v.y;
    is >> std::ws;
    if (is.peek() == ']') {
      is.get();                     // consume ']'
    }
  } else {
    is >> v.x >> v.y;
  }
  return is;
}

Vector2D operator-(const Point2D & head, const Point2D & tail)
{
  return {head.x - tail.x, head.y - tail.y};
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  return {tail.x + disp.x, tail.y + disp.y};
}

Vector2D & Vector2D::operator+=(const Vector2D & rhs)
{
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
{
  return lhs += rhs;
}

Vector2D & Vector2D::operator-=(const Vector2D & rhs)
{
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}

Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
{
  return lhs -= rhs;
}

Vector2D & Vector2D::operator*=(const double s)
{
  x *= s;
  y *= s;
  return *this;
}

Vector2D operator*(Vector2D v, const double s)
{
  return v *= s;
}

Vector2D operator*(const double s, Vector2D v)
{
  return v *= s;
}

double dot(Vector2D v1, Vector2D v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

double magnitude(Vector2D v)
{
  return std::sqrt(dot(v, v));
}

double angle(Vector2D v1, Vector2D v2)
{
  const auto m1 = magnitude(v1);
  const auto m2 = magnitude(v2);
  if (almost_equal(m1, 0.0) || almost_equal(m2, 0.0)) {
    return 0.0;
  }
        // Shortest angle formula
  return std::acos(dot(v1, v2) / (m1 * m2));
}

Vector2D normalize(Vector2D in)
{
  const auto mag = magnitude(in);
        // Guidelines: Use double precision literals
  if (almost_equal(mag, 0.0)) {
    throw std::invalid_argument("Cannot normalize a zero vector.");
  }
  return {in.x / mag, in.y / mag};
}
}
