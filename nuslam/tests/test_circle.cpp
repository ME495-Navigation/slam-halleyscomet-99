/// \file
/// \brief Unit tests for the Pratt circle fitting algorithm (fit_circle).
///
/// Test data and expected results are taken from the algorithm authors' website:
/// https://people.cas.uab.edu/~mosya/cl/CPPcircle.html
/// All results should be within 1e-4 of the reference values.

#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "turtlelib/geometry2d.hpp"

/// \brief Tolerance for comparing fitted circle parameters to reference values.
static constexpr double TOL = 1e-4;

// ---------------------------------------------------------------------------
// Test 1 — from the algorithm authors' reference page
// Inputs:  {(1,7),(2,6),(5,8),(7,7),(9,5),(3,7)}
// Expected: centre (4.615482, 2.807354), radius 4.8275
// ---------------------------------------------------------------------------
TEST(CircleFitting, Test1)
{
  const std::vector<turtlelib::Point2D> cluster = {
    {1.0, 7.0},
    {2.0, 6.0},
    {5.0, 8.0},
    {7.0, 7.0},
    {9.0, 5.0},
    {3.0, 7.0}
  };

  const auto circle = turtlelib::fit_circle(cluster);

  EXPECT_NEAR(circle.center.x, 4.615482, TOL);
  EXPECT_NEAR(circle.center.y, 2.807354, TOL);
  EXPECT_NEAR(circle.radius, 4.8275, TOL);
}

// ---------------------------------------------------------------------------
// Test 2 — from the algorithm authors' reference page
// Inputs:  {(-1,0),(-0.3,-0.06),(0.3,0.1),(1,0)}
// Expected: centre (0.4908357, -22.15212), radius 22.17979
// ---------------------------------------------------------------------------
TEST(CircleFitting, Test2)
{
  const std::vector<turtlelib::Point2D> cluster = {
    {-1.0, 0.0},
    {-0.3, -0.06},
    {0.3, 0.1},
    {1.0, 0.0}
  };

  const auto circle = turtlelib::fit_circle(cluster);

  EXPECT_NEAR(circle.center.x, 0.4908357, TOL);
  EXPECT_NEAR(circle.center.y, -22.15212, TOL);
  EXPECT_NEAR(circle.radius, 22.17979, TOL);
}

// ---------------------------------------------------------------------------
// Test 3 — degenerate: fewer than 3 points must throw
// ---------------------------------------------------------------------------
TEST(CircleFitting, ThrowsOnTooFewPoints)
{
  const std::vector<turtlelib::Point2D> cluster = {
    {0.0, 0.0},
    {1.0, 1.0}
  };

  EXPECT_THROW(turtlelib::fit_circle(cluster), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Test 4 — exact circle: 4 points on a known circle should recover it
// Circle: centre (1, 2), radius 3  =>  points on axes of that circle
// ---------------------------------------------------------------------------
TEST(CircleFitting, ExactCircle)
{
  // Points exactly on the circle x=(1+3cosθ), y=(2+3sinθ)
  const std::vector<turtlelib::Point2D> cluster = {
    {4.0, 2.0},    // θ = 0
    {1.0, 5.0},    // θ = π/2
    {-2.0, 2.0},   // θ = π
    {1.0, -1.0}    // θ = 3π/2
  };

  const auto circle = turtlelib::fit_circle(cluster);

  EXPECT_NEAR(circle.center.x, 1.0, TOL);
  EXPECT_NEAR(circle.center.y, 2.0, TOL);
  EXPECT_NEAR(circle.radius, 3.0, TOL);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
