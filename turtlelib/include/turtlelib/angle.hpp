#ifndef TURTLELIB_ANGLE_HPP_INCLUDE_GUARD
#define TURTLELIB_ANGLE_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles
/// NOTE: Include any needed header files here
#include <cmath>
#include <numbers>

namespace turtlelib
{
    /// \brief Approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 A number to compare
    /// \param d2 A second number to compare
    /// \param epsilon Absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
{
    // this works, but can be simplified
  double diff = d1 - d2;
  if (diff < 0) {
    diff = -diff;
  }
  return diff < epsilon;
}

    /// \brief Convert degrees to radians
    /// \param deg Angle in degrees
    /// \returns The equivalent angle in radians
constexpr double deg2rad(double deg)
{
        // HINT: C++20 #include<numbers> defines standard values
        // for many mathematical constants. Prior to C++20 you
        // would need to define your own constant for pi.
        // You should use the standardized value now
  return deg * std::numbers::pi / 180.0;
}

    /// \brief Convert radians to degrees
    /// \param rad  Angle in radians
    /// \returns The equivalent angle in degrees
constexpr double rad2deg(double rad)
{
  return rad * 180.0 / std::numbers::pi;
}

    /// \brief Wrap an angle to (-PI, PI]
    /// \param rad Angle in radians
    /// \return An equivalent angle the range (-PI, PI]
constexpr double normalize_angle(double rad)
{
        // NOTE: You will receive partial credit only if this function uses loops.
    // fmod could help
  while (rad > std::numbers::pi) {
    rad -= 2.0 * std::numbers::pi;
  }
  while (rad <= -std::numbers::pi) {
    rad += 2.0 * std::numbers::pi;
  }
  return rad;
}

    // --- Static Assertions (Compile-time tests) ---

    // 1. almost_equal tests
static_assert(almost_equal(0, 0), "is_zero failed");
static_assert(almost_equal(1.0, 1.00000000000001, 1.0e-10), "almost_equal true failed");
static_assert(!almost_equal(1.0, 1.1, 1.0e-10), "almost_equal false failed");
static_assert(!almost_equal(-1.0, 1.0), "neg/pos equal failed");
static_assert(almost_equal(-0.0, 0.0), "neg/pos zero failed");

    // 2. deg2rad tests
static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad 0 failed");
static_assert(almost_equal(deg2rad(180.0), std::numbers::pi), "deg2rad 180 failed");
static_assert(almost_equal(deg2rad(-90.0), -std::numbers::pi / 2.0), "deg2rad -90 failed");
static_assert(almost_equal(deg2rad(360.0), 2.0 * std::numbers::pi), "deg2rad 360 failed");
static_assert(almost_equal(deg2rad(45.0), std::numbers::pi / 4.0), "deg2rad 45 failed");

    // 3. rad2deg tests
static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg 0 failed");
static_assert(almost_equal(rad2deg(std::numbers::pi), 180.0), "rad2deg pi failed");
static_assert(almost_equal(rad2deg(-std::numbers::pi / 4.0), -45.0), "rad2deg -pi/4 failed");
static_assert(almost_equal(rad2deg(2.0 * std::numbers::pi), 360.0), "rad2deg 2pi failed");
static_assert(almost_equal(rad2deg(-std::numbers::pi), -180.0), "rad2deg -pi failed");

    // 4. normalize_angle tests
static_assert(almost_equal(normalize_angle(0.0), 0.0), "norm_angle 0 failed");
static_assert(almost_equal(normalize_angle(std::numbers::pi), std::numbers::pi),
    "norm_angle pi failed");
static_assert(almost_equal(normalize_angle(-std::numbers::pi), std::numbers::pi),
    "norm_angle -pi failed");
static_assert(almost_equal(normalize_angle(6.0 * std::numbers::pi), 0.0),
    "norm_angle > 5pi failed");
static_assert(almost_equal(normalize_angle(-6.0 * std::numbers::pi), 0.0),
    "norm_angle < -5pi failed");
static_assert(almost_equal(normalize_angle(-std::numbers::pi / 4.0), -std::numbers::pi / 4.0),
    "norm_angle -pi/4 failed");
static_assert(almost_equal(normalize_angle(1.5 * std::numbers::pi), -0.5 * std::numbers::pi),
    "norm_angle 3pi/2 failed");
static_assert(almost_equal(normalize_angle(-2.5 * std::numbers::pi), -0.5 * std::numbers::pi),
    "norm_angle -5pi/2 failed");
static_assert(almost_equal(normalize_angle(std::numbers::pi + 0.0001), -std::numbers::pi + 0.0001),
    "norm_angle wrap failed");


}
#endif
