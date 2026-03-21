#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD

/// \file
/// \brief Two-dimensional geometric primitives and other mathematical objects

#include <iosfwd>
#include <format>
#include <vector>

namespace turtlelib
{
/// \brief a 2-Dimensional Point
struct Point2D
{
  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;
};

/// \brief Input a 2 dimensional point
/// \param is - An istream from which to read
/// \param p [out] - The Point2D object that will store the input
/// \returns A reference to is.
std::istream & operator>>(std::istream & is, Point2D & p);

/// \brief A 2-Dimensional Vector
struct Vector2D
{
  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;

  /// \brief Add a vector to this vector
  /// \param rhs - the vector to add
  /// \return a reference to the modified vector
  Vector2D & operator+=(const Vector2D & rhs);

  /// \brief Subtract a vector from this vector
  /// \param rhs - the vector to subtract
  /// \return a reference to the modified vector
  Vector2D & operator-=(const Vector2D & rhs);

  /// \brief Multiply this vector by a scalar
  /// \param rhs - the scalar to multiply by
  /// \return a reference to the modified vector
  Vector2D & operator*=(const double rhs);
};

/// \brief Represent a 2D Circle
struct Circle
{
  /// \brief The center of the circle
  Point2D center;
  /// \brief The radius of the circle
  double radius = 0.0;
};

/// \brief Adding two vectors together
/// \param lhs - the left hand vector
/// \param rhs - the right hand vector
/// \return the sum of the two vectors
Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

/// \brief Subtracting two vectors
/// \param lhs - the left hand vector
/// \param rhs - the right hand vector
/// \return the difference of the two vectors
Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

/// \brief Multiplying a vector by a scalar
/// \param lhs - the vector to scale
/// \param rhs - the scalar
/// \return the scaled vector
Vector2D operator*(Vector2D lhs, const double rhs);

/// \brief Multiplying a scalar by a vector
/// \param lhs - the scalar
/// \param rhs - the vector to scale
/// \return the scaled vector
Vector2D operator*(const double lhs, Vector2D rhs);

/// \brief Compute the dot product of two vectors
/// \param v1 - the first vector
/// \param v2 - the second vector
/// \returns the dot product
double dot(Vector2D v1, Vector2D v2);

/// \brief Compute the magnitude of a vector
/// \param v - the vector
/// \returns the magnitude
double magnitude(Vector2D v);

/// \brief Compute the Euclidean distance between two points
/// \param p1 - the first point
/// \param p2 - the second point
/// \returns the distance
double distance(Point2D p1, Point2D p2);

/// \brief Compute the shortest angle between two vectors
/// \param v1 - the first vector
/// \param v2 - the second vector
/// \returns the angle in radians
double angle(Vector2D v1, Vector2D v2);

/// \brief Subtracting one point from another yields a vector
/// \param head - point corresponding to the head of the vector
/// \param tail - point corresponding to the tail of the vector
/// \return a vector that points from tail to head
Vector2D operator-(const Point2D & head, const Point2D & tail);

/// \brief Adding a vector to a point yields a new point displaced by the vector
/// \param tail - The origin of the vector's tail
/// \param disp - The displacement vector
/// \return the point reached by displacing by disp from tail
Point2D operator+(const Point2D & tail, const Vector2D & disp);

/// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
/// \param os - stream to output to
/// \param v - the vector to print
/// \returns the ostream
std::ostream & operator<<(std::ostream & os, const Vector2D & v);

/// \brief input a 2 dimensional vector
/// \param is - An istream from which to read
/// \param v [out] - output vector
/// \returns a reference to the istream
std::istream & operator>>(std::istream & is, Vector2D & v);

/// \brief Return a unit vector in the direction of v
/// \param in - The vector to normalize
/// \return The normalized vector.
/// \throws std::invalid_argument if in is the zero vector
Vector2D normalize(Vector2D in);

/// \brief Fit a circle to a cluster of 2D points using the Pratt Method
/// \param cluster - A vector of points belonging to a single potential landmark
/// \return A Circle object containing the estimated center and radius
Circle fit_circle(const std::vector<Point2D> & cluster);

}

/// \cond
/// \brief A Formatter for 2D points. Output is (x, y)
template<class CharT>
struct std::formatter<turtlelib::Point2D, CharT>: std::formatter<double, CharT>
{
  constexpr auto parse(std::format_parse_context & ctx)
  {
    return std::formatter<double, CharT>::parse(ctx);
  }

  template<typename FormatContext>
  auto format(const turtlelib::Point2D & p, FormatContext & ctx) const
  {
    auto it = ctx.out();
    it = std::format_to(it, "(");
    ctx.advance_to(it);
    it = std::formatter<double, CharT>::format(p.x, ctx);
    it = std::format_to(it, ", ");
    ctx.advance_to(it);
    it = std::formatter<double, CharT>::format(p.y, ctx);
    return std::format_to(it, ")");
  }
};

/// \brief A Formatter for 2D vectors. Output is [x, y]
template<class CharT>
struct std::formatter<turtlelib::Vector2D, CharT>: std::formatter<double, CharT>
{
  constexpr auto parse(std::format_parse_context & ctx)
  {
    return std::formatter<double, CharT>::parse(ctx);
  }

  template<typename FormatContext>
  auto format(const turtlelib::Vector2D & v, FormatContext & ctx) const
  {
    auto it = ctx.out();
    it = std::format_to(it, "[");
    ctx.advance_to(it);
    it = std::formatter<double, CharT>::format(v.x, ctx);
    it = std::format_to(it, ", ");
    ctx.advance_to(it);
    it = std::formatter<double, CharT>::format(v.y, ctx);
    return std::format_to(it, "]");
  }
};
/// \endcond

#endif