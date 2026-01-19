#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include "turtlelib/se2d.hpp"

using namespace turtlelib;

TEST_CASE("Transform2D default constructor (identity)", "[transform]") // Your Name
{
    Transform2D tf;
    CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-12));
}

TEST_CASE("Transform2D pure translation constructor", "[transform]") // Your Name
{
    Transform2D tf({3.5, -2.0});
    CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(3.5, 1e-12));
    CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-2.0, 1e-12));
}

TEST_CASE("Transform2D inverse", "[transform]") // Your Name
{
    // Rotate 90 deg and translate (1, 1)
    Transform2D tf({1.0, 1.0}, std::numbers::pi / 2.0);
    Transform2D inv = tf.inv();
    
    CHECK_THAT(inv.rotation(), Catch::Matchers::WithinAbs(-std::numbers::pi / 2.0, 1e-12));
    // R^T * -p = [[0, 1], [-1, 0]] * [-1, -1]^T = [-1, 1]
    CHECK_THAT(inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    CHECK_THAT(inv.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-12));
}

TEST_CASE("Transform2D composition (operator*=)", "[transform]") // Your Name
{
    Transform2D tf1({1.0, 0.0}, std::numbers::pi / 2.0);
    Transform2D tf2({1.0, 0.0}, 0.0);
    
    tf1 *= tf2; // Move 1 unit in the local x-direction of tf1
    
    CHECK_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(std::numbers::pi / 2.0, 1e-12));
    // Since tf1 is rotated 90 deg, its local x is world y.
    CHECK_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-12));
    CHECK_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-12));
}

TEST_CASE("Twist2D transformation (Adjoint)", "[transform]") // Your Name
{
    Transform2D tf({1.0, 1.0}, std::numbers::pi / 2.0);
    Twist2D tw{1.0, 1.0, 0.0}; // Rotate 1 rad/s, move 1m/s in x
    
    Twist2D result = tf(tw);
    
    CHECK_THAT(result.omega, Catch::Matchers::WithinAbs(1.0, 1e-12));
    // x_new = y*w + cos(th)*vx - sin(th)*vy = 1*1 + 0*1 - 1*0 = 1.0
    // y_new = -x*w + sin(th)*vx + cos(th)*vy = -1*1 + 1*1 + 0*0 = 0.0
    CHECK_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-12));
    CHECK_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-12));
}

TEST_CASE("Transform2D operator>> (bracket format)", "[transform]") // Your Name
{
    std::stringstream ss("{90 deg, 1.5, 2.5}");
    Transform2D tf;
    ss >> tf;
    
    CHECK_THAT(tf.rotation(), Catch::Matchers::WithinAbs(std::numbers::pi / 2.0, 1e-12));
    CHECK_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.5, 1e-12));
    CHECK_THAT(tf.translation().y, Catch::Matchers::WithinAbs(2.5, 1e-12));
}