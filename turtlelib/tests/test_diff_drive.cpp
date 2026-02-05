/// \file
/// \brief Unit tests for differential drive kinematics.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <numbers>
#include "turtlelib/diff_drive.hpp"

using turtlelib::DiffDrive;
using turtlelib::WheelPositions;
using turtlelib::WheelVelocities;
using turtlelib::Twist2D;
using turtlelib::almost_equal;

TEST_CASE("DiffDrive kinematics", "[diff_drive]") // Chenwan Zhong
{
    // Standard parameters for a TurtleBot3 Burger
    const auto track = 0.16;
    const auto radius = 0.033;
    DiffDrive robot(track, radius);

    SECTION("Forward Kinematics: Pure translation") {
        // Robot moves forward: both wheels turn 1 radian
        // Distance = r * phi = 0.033 * 1.0 = 0.033m
        robot.forward_kinematics({1.0, 1.0});
        const auto q = robot.configuration();
        
        CHECK_THAT(q.translation().x, Catch::Matchers::WithinAbs(0.033, 1e-5));
        CHECK_THAT(q.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        CHECK_THAT(q.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    SECTION("Inverse Kinematics: Pure translation") {
        // Desired v_x = 0.033 m/s, omega = 0.0
        // Expected wheel velocities = v/r = 0.033 / 0.033 = 1.0 rad/s
        Twist2D cmd{0.0, 0.033, 0.0};
        const auto wheel_vels = robot.inverse_kinematics(cmd);
        
        CHECK_THAT(wheel_vels.left, Catch::Matchers::WithinAbs(1.0, 1e-5));
        CHECK_THAT(wheel_vels.right, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }

    SECTION("Forward Kinematics: Pure rotation") {
        // Wheels turn in opposite directions: left -1.0, right 1.0
        // Delta theta = (r/D) * (dr - dl) = (0.033 / 0.16) * (1.0 - (-1.0)) = 0.4125 rad
        robot.forward_kinematics({-1.0, 1.0});
        const auto q = robot.configuration();
        
        CHECK_THAT(q.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
        CHECK_THAT(q.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        CHECK_THAT(q.rotation(), Catch::Matchers::WithinAbs(0.4125, 1e-5));
    }

    SECTION("Inverse Kinematics: Pure rotation") {
        // Desired omega = 0.4125 rad/s, v_x = 0.0
        Twist2D cmd{0.4125, 0.0, 0.0};
        const auto wheel_vels = robot.inverse_kinematics(cmd);
        
        CHECK_THAT(wheel_vels.left, Catch::Matchers::WithinAbs(-1.0, 1e-5));
        CHECK_THAT(wheel_vels.right, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }

    SECTION("Forward Kinematics: Circular arc") {
        // Case: Only right wheel moves 1.0 radian
        // This makes the robot follow an arc
        robot.forward_kinematics({0.0, 1.0});
        const auto q = robot.configuration();
        
        // Detailed math:
        // d_theta = (0.033 / 0.16) * (1.0 - 0.0) = 0.20625 rad
        // d_x = (0.033 / 2.0) * (1.0 + 0.0) = 0.0165 m
        // integrate_twist uses these to find (x, y) relative pose
        CHECK_THAT(q.rotation(), Catch::Matchers::WithinAbs(0.20625, 1e-5));
        // Use your earlier integrate_twist logic to verify x and y
        CHECK(q.translation().x > 0.0);
        CHECK(q.translation().y > 0.0);
    }

    SECTION("Inverse Kinematics: Circular arc") {
        // Move along a circle of radius R=1.0 at v=1.0 m/s
        // omega = v/R = 1.0 rad/s
        Twist2D cmd{1.0, 1.0, 0.0};
        const auto vels = robot.inverse_kinematics(cmd);
        
        // Left: (1/r)*(v - D/2*w) = (1/0.033)*(1.0 - 0.08) = 0.92 / 0.033 = 27.8787
        // Right: (1/r)*(v + D/2*w) = (1/0.033)*(1.0 + 0.08) = 1.08 / 0.033 = 32.7272
        CHECK_THAT(vels.left, Catch::Matchers::WithinAbs(27.878787, 1e-4));
        CHECK_THAT(vels.right, Catch::Matchers::WithinAbs(32.727272, 1e-4));
    }

    SECTION("Impossible twist: non-zero y velocity") {
        // Diff drive cannot move sideways
        Twist2D cmd{0.0, 1.0, 1.0}; // v_y = 1.0
        CHECK_THROWS_AS(robot.inverse_kinematics(cmd), std::logic_error);
    }
}