/// \file
/// \brief Unit tests for 2D geometry primitives and operations.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <stdexcept>
#include "turtlelib/geometry2d.hpp"

using turtlelib::Point2D;
using turtlelib::Vector2D;

TEST_CASE("Point2D subtraction (yields Vector2D)", "[geometry2d]") {
    const Point2D p1{5.0, 10.0};
    const Point2D p2{2.0, 4.0};
    const Vector2D v = p1 - p2;
    CHECK_THAT(v.x, Catch::Matchers::WithinAbs(3.0, 1e-12));
    CHECK_THAT(v.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
}

TEST_CASE("Point2D + Vector2D (yields Point2D)", "[geometry2d]") {
    const Point2D p{1.0, 2.0};
    const Vector2D v{3.0, 4.0};
    const Point2D result = p + v;
    CHECK_THAT(result.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
    CHECK_THAT(result.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
}

TEST_CASE("Vector2D addition and assignment (+=)", "[geometry2d]") {
    Vector2D v1{1.0, 2.0};
    const Vector2D v2{3.0, 4.0};
    v1 += v2;
    CHECK_THAT(v1.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
    CHECK_THAT(v1.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
}

TEST_CASE("Vector2D addition (+)", "[geometry2d]") {
    const Vector2D v1{1.0, 2.0};
    const Vector2D v2{3.0, 4.0};
    const auto v3 = v1 + v2;
    CHECK_THAT(v3.x, Catch::Matchers::WithinAbs(4.0, 1e-12));
    CHECK_THAT(v3.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
}

TEST_CASE("Vector2D subtraction and assignment (-=)", "[geometry2d]") {
    Vector2D v1{5.0, 5.0};
    const Vector2D v2{2.0, 3.0};
    v1 -= v2;
    CHECK_THAT(v1.x, Catch::Matchers::WithinAbs(3.0, 1e-12));
    CHECK_THAT(v1.y, Catch::Matchers::WithinAbs(2.0, 1e-12));
}

TEST_CASE("Vector2D multiplication by scalar", "[geometry2d]") {
    const Vector2D v{1.0, 2.0};
    
    SECTION("Vector * Scalar") {
        const auto res = v * 2.5;
        CHECK_THAT(res.x, Catch::Matchers::WithinAbs(2.5, 1e-12));
        CHECK_THAT(res.y, Catch::Matchers::WithinAbs(5.0, 1e-12));
    }
    
    SECTION("Scalar * Vector") {
        const auto res = 2.5 * v;
        CHECK_THAT(res.x, Catch::Matchers::WithinAbs(2.5, 1e-12));
        CHECK_THAT(res.y, Catch::Matchers::WithinAbs(5.0, 1e-12));
    }

    SECTION("Compound assignment (*=)") {
        Vector2D v_mutable{1.0, 2.0};
        v_mutable *= 3.0;
        CHECK_THAT(v_mutable.x, Catch::Matchers::WithinAbs(3.0, 1e-12));
        CHECK_THAT(v_mutable.y, Catch::Matchers::WithinAbs(6.0, 1e-12));
    }
}

TEST_CASE("Vector2D dot product", "[geometry2d]") {
    const Vector2D v1{1.0, 2.0};
    const Vector2D v2{3.0, 4.0};
    const auto d = dot(v1, v2); // 1*3 + 2*4 = 11
    CHECK_THAT(d, Catch::Matchers::WithinAbs(11.0, 1e-12));
}

TEST_CASE("Vector2D magnitude", "[geometry2d]") {
    const Vector2D v{3.0, 4.0};
    CHECK_THAT(magnitude(v), Catch::Matchers::WithinAbs(5.0, 1e-12));
}

TEST_CASE("Vector2D angle", "[geometry2d]") {
    const Vector2D v1{1.0, 0.0};
    const Vector2D v2{0.0, 1.0};
    const auto ang = angle(v1, v2);
    CHECK_THAT(ang, Catch::Matchers::WithinAbs(1.57079632679, 1e-10)); // PI/2
}

TEST_CASE("Vector2D output operator<<", "[geometry2d]") {
    const Vector2D v{1.5, -2.5};
    std::stringstream ss;
    ss << v;
    CHECK(ss.str() == "[1.5 -2.5]"); // 确保这和你 operator<< 实现的格式一致
}

TEST_CASE("Vector2D input operator>>", "[geometry2d]") {
    SECTION("Standard format [x y]") {
        std::stringstream ss("[1.1 2.2]");
        Vector2D v;
        ss >> v;
        CHECK_THAT(v.x, Catch::Matchers::WithinAbs(1.1, 1e-12));
        CHECK_THAT(v.y, Catch::Matchers::WithinAbs(2.2, 1e-12));
    }
}

TEST_CASE("Vector2D normalization", "[geometry2d]") {
    SECTION("Normal vector") {
        const Vector2D v{3.0, 4.0};
        const Vector2D unit = normalize(v);
        CHECK_THAT(unit.x, Catch::Matchers::WithinAbs(0.6, 1e-12));
        CHECK_THAT(unit.y, Catch::Matchers::WithinAbs(0.8, 1e-12));
    }
    SECTION("Zero vector throws exception") {
        const Vector2D v{0.0, 0.0};
        CHECK_THROWS_AS(turtlelib::normalize(v), std::invalid_argument);
    }
}
