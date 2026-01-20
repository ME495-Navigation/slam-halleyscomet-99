#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include "turtlelib/geometry2d.hpp"

using turtlelib::Point2D;
using turtlelib::Vector2D;

TEST_CASE("Point2D subtraction (yields Vector2D)", "[geometry2d]") {
    Point2D p1{5.0, 10.0};
    Point2D p2{2.0, 4.0};
    Vector2D v = p1 - p2;
    CHECK_THAT(v.x, Catch::Matchers::WithinAbs(3.0, 1e-12));
    CHECK_THAT(v.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
}

TEST_CASE("Point2D + Vector2D (yields Point2D)", "[geometry2d]") {
    Point2D p{1.0, 2.0};
    Vector2D v{3.0, 4.0};
    Point2D result = p + v;
    CHECK_THAT(result.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
    CHECK_THAT(result.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
}

TEST_CASE("Vector2D output operator<<", "[geometry2d]") {
    Vector2D v{1.5, -2.5};
    std::stringstream ss;
    ss << v;
    CHECK(ss.str() == "[1.5, -2.5]");
}

TEST_CASE("Vector2D input operator>>", "[geometry2d]") {
    SECTION("Standard format [x, y]") {
        std::stringstream ss("[1.1, 2.2]");
        Vector2D v;
        ss >> v;
        CHECK_THAT(v.x, Catch::Matchers::WithinAbs(1.1, 1e-12));
        CHECK_THAT(v.y, Catch::Matchers::WithinAbs(2.2, 1e-12));
    }
    SECTION("Alternative format x y") {
        std::stringstream ss("3.3 4.4");
        Vector2D v;
        ss >> v;
        CHECK_THAT(v.x, Catch::Matchers::WithinAbs(3.3, 1e-12));
        CHECK_THAT(v.y, Catch::Matchers::WithinAbs(4.4, 1e-12));
    }
}

TEST_CASE("Point2D input operator>>", "[geometry2d]") {
    SECTION("Standard format (x, y)") {
        std::stringstream ss("(0.5, 0.5)");
        Point2D p;
        ss >> p;
        CHECK_THAT(p.x, Catch::Matchers::WithinAbs(0.5, 1e-12));
        CHECK_THAT(p.y, Catch::Matchers::WithinAbs(0.5, 1e-12));
    }
}

TEST_CASE("Vector2D normalization", "[geometry2d]") {
    SECTION("Normal vector") {
        Vector2D v{3.0, 4.0}; // Length is 5
        Vector2D unit = normalize(v);
        CHECK_THAT(unit.x, Catch::Matchers::WithinAbs(0.6, 1e-12));
        CHECK_THAT(unit.y, Catch::Matchers::WithinAbs(0.8, 1e-12));
    }
    SECTION("Zero vector throws exception") {
        Vector2D v{0.0, 0.0};
        CHECK_THROWS_AS(turtlelib::normalize(v), std::invalid_argument);
    }
}
