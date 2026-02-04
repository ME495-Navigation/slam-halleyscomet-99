#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP

/// \file
/// \brief Two-dimensional rigid body transformations and twists.

#include <iosfwd>
#include <format>
#include <string>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{
    /// \brief represent a 2-Dimensional twist
struct Twist2D
{
        /// \brief the angular velocity
  double omega = 0.0;

        /// \brief the linear x velocity
  double x = 0.0;

        /// \brief the linear y velocity
  double y = 0.0;

        /// \brief multiply a twist by a scalar
        /// \param s - the scalar to multiply by
        /// \return a reference to the updated twist
  Twist2D & operator*=(const double s);
};

    /// \brief multiply a twist by a scalar
    /// \param lhs - the twist
    /// \param rhs - the scalar
    /// \return a scaled twist
Twist2D operator*(Twist2D lhs, const double rhs);

    /// \brief multiply a twist by a scalar
    /// \param lhs - the scalar
    /// \param rhs - the twist
    /// \return a scaled twist
Twist2D operator*(const double lhs, Twist2D rhs);

    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
std::istream & operator>>(std::istream & is, Twist2D & tw);

    /// \brief a rigid body transformation in 2 dimensions
class Transform2D
{
public:
        /// \brief Create an identity transformation
  Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
  explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
  explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
  Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a 2D Point
        /// \param p the point to transform
        /// \return a point in the new coordinate system
  Point2D operator()(Point2D p) const;

        /// \brief apply a transformation to a 2D Vector
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
  Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D (using the adjoint)
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
  Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
  Transform2D inv() const;

        /// \brief compose this transform with another
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
  Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
  Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
  double rotation() const;

        /// \brief Formatter friend declaration
  template<class CharT>
  friend struct std::formatter;

private:
  double x_ = 0.0;                 ///< Translation in x
  double y_ = 0.0;                 ///< Translation in y
  double theta_ = 0.0;             ///< Rotation in radians
};

    /// \brief Read a transformation from stdin
std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief Compute the transformation corresponding to a constant twist for one time unit
    /// \param tw - the constant twist
    /// \return the resulting transformation
Transform2D integrate_twist(Twist2D tw);

} // namespace turtlelib

/// \cond
/// \brief Formatter for Transform2D
template<class CharT>
struct std::formatter<turtlelib::Transform2D, CharT>: std::formatter<double, CharT>
{
    /// \brief The unit type to display: 'R' for rad, 'D' for deg
  char unit_type = '\0';

    /// \brief Parse the format specifier
    /// \param ctx The parse context
    /// \return The iterator to the end of the format specifier
  template<typename ParseContext>
  constexpr auto parse(ParseContext & ctx)
  {
    auto it = ctx.begin();
    if (it != ctx.end() && (*it == 'R' || *it == 'D')) {
      unit_type = *it++;
    }
    return std::formatter<double, CharT>::parse(ctx);
  }

    /// \brief Format the Transform2D
    /// \param tf The transform to format
    /// \param ctx The format context
    /// \return The iterator to the end of the formatted string
  template<typename FormatContext>
  auto format(const turtlelib::Transform2D & tf, FormatContext & ctx) const
  {
    auto it = ctx.out();
    double rot = tf.rotation();
    std::string unit = "";
    if (unit_type == 'R') {
      unit = " rad";
    } else if (unit_type == 'D') {
      rot = turtlelib::rad2deg(rot);
      unit = " deg";
    }

    it = std::format_to(it, "{{");
    ctx.advance_to(it);
    it = std::formatter<double, CharT>::format(rot, ctx);
    it = std::format_to(it, "{}, {}, {}}}", unit, tf.translation().x, tf.translation().y);
    return it;
  }
};

/// \brief Formatter for Twist2D
template<class CharT>
struct std::formatter<turtlelib::Twist2D, CharT>: std::formatter<double, CharT>
{
    /// \brief The unit type to display: 'R' for rad/s, 'D' for deg/s
  char unit_type = '\0';

    /// \brief Parse the format specifier
  template<typename ParseContext>
  constexpr auto parse(ParseContext & ctx)
  {
    auto it = ctx.begin();
    if (it != ctx.end() && (*it == 'R' || *it == 'D')) {
      unit_type = *it++;
    }
    return std::formatter<double, CharT>::parse(ctx);
  }

    /// \brief Format the Twist2D
    /// \param tw The twist to format
    /// \param ctx The format context
    /// \return The iterator to the end of the formatted string
  template<typename FormatContext>
  auto format(const turtlelib::Twist2D & tw, FormatContext & ctx) const
  {
    auto it = ctx.out();
    double rot = tw.omega;
    std::string unit = "";
    if (unit_type == 'R') {
      unit = " rad/s";
    } else if (unit_type == 'D') {
      rot = turtlelib::rad2deg(rot);
      unit = " deg/s";
    }

    it = std::format_to(it, "[");
    ctx.advance_to(it);
    it = std::formatter<double, CharT>::format(rot, ctx);
    it = std::format_to(it, "{}, {}, {}]", unit, tw.x, tw.y);
    return it;
  }
};
/// \endcond

#endif
