/// \file
/// \brief Unit tests for 2D transformations and twist integration.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <numbers>
#include "turtlelib/se2d.hpp"

using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Vector2D;
using turtlelib::Point2D;

TEST_CASE("Transform2D default constructor (identity)", "[transform]") // Chenwan Halley Zhong
{
    Transform2D tf;
    CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-12));
}

TEST_CASE("Transform2D pure translation constructor", "[transform]") // Chenwan Halley Zhong
{
    Transform2D tf(Vector2D{3.5, -2.0});
    CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(3.5, 1e-12));
    CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-2.0, 1e-12));
}

TEST_CASE("Transform2D inverse", "[transform]") // Chenwan Halley Zhong
{
    // Rotate 90 deg and translate (1, 1)
    Transform2D tf(Vector2D{1.0, 1.0}, std::numbers::pi / 2.0);
    Transform2D inv = tf.inv();

    CHECK_THAT(inv.rotation(), Catch::Matchers::WithinAbs(-std::numbers::pi / 2.0, 1e-12));
    // R^T * -p = [[0, 1], [-1, 0]] * [-1, -1]^T = [-1, 1]
    CHECK_THAT(inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    CHECK_THAT(inv.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-12));
}

TEST_CASE("Transform2D composition (operator*=)", "[transform]") // Chenwan Halley Zhong
{
    Transform2D tf1(Vector2D{1.0, 0.0}, std::numbers::pi / 2.0);
    Transform2D tf2(Vector2D{1.0, 0.0}, 0.0);

    tf1 *= tf2; // Move 1 unit in the local x-direction of tf1

    CHECK_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(std::numbers::pi / 2.0, 1e-12));
    // Since tf1 is rotated 90 deg, its local x is world y.
    CHECK_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-12));
    CHECK_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-12));
}

TEST_CASE("Twist2D transformation (Adjoint)", "[transform]") // Chenwan Halley Zhong
{
    Transform2D tf(Vector2D{1.0, 1.0}, std::numbers::pi / 2.0);
    Twist2D tw{1.0, 1.0, 0.0}; // Rotate 1 rad/s, move 1m/s in x

    Twist2D result = tf(tw);

    CHECK_THAT(result.omega, Catch::Matchers::WithinAbs(1.0, 1e-12));
    // x_new = y*w + cos(th)*vx - sin(th)*vy = 1*1 + 0*1 - 1*0 = 1.0
    // y_new = -x*w + sin(th)*vx + cos(th)*vy = -1*1 + 1*1 + 0*0 = 0.0
    CHECK_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-12));
    CHECK_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-12));
}

TEST_CASE("Twist2D scalar multiplication (operator*= and *)", "[twist]") // Chenwan Halley Zhong
{
    Twist2D tw{1.0, 2.0, 3.0};

    SECTION("operator*=") {
        tw *= 2.0;
        CHECK_THAT(tw.omega, Catch::Matchers::WithinAbs(2.0, 1e-12));
        CHECK_THAT(tw.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
        CHECK_THAT(tw.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
    }

    SECTION("operator* (right and left)") {
        const auto tw2 = tw * 0.5;
        const auto tw3 = 10.0 * tw;
        CHECK_THAT(tw2.x, Catch::Matchers::WithinAbs(1.0, 1e-12));
        CHECK_THAT(tw3.y, Catch::Matchers::WithinAbs(30.0, 1e-12));
    }
}

TEST_CASE("integrate_twist", "[se2d]") // Chenwan Halley Zhong
{
    SECTION("Pure translation") {
        Twist2D tw{0.0, 1.5, -2.0};
        const auto tf = turtlelib::integrate_twist(tw);
        CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.5, 1e-12));
        CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-2.0, 1e-12));
        CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-12));
    }

    SECTION("Pure rotation") {
        const auto pi = std::numbers::pi;
        Twist2D tw{pi / 2.0, 0.0, 0.0};
        const auto tf = turtlelib::integrate_twist(tw);
        CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-12));
        CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-12));
        CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(pi / 2.0, 1e-12));
    }

    SECTION("Simultaneous translation and rotation (Circular Arc)") {
        const auto pi = std::numbers::pi;
        // Robot rotates 180 degrees while having forward velocity 1.0
        // Radius of circle r = v/w = 1.0 / pi.
        // After 180 deg, it should be at (0, 2r) = (0, 2/pi)
        Twist2D tw{pi, 1.0, 0.0};
        const auto tf = turtlelib::integrate_twist(tw);
        CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-12));
        CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(2.0 / pi, 1e-12));
        CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(pi, 1e-12));
    }
}

TEST_CASE("Transform2D operator>> (bracket format)", "[transform]") // Chenwan Halley Zhong
{
    std::stringstream ss("{90 deg, 1.5, 2.5}");
    Transform2D tf;
    ss >> tf;

    CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(std::numbers::pi / 2.0, 1e-12));
    CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.5, 1e-12));
    CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(2.5, 1e-12));
}
