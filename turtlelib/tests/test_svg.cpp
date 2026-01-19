#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include "turtlelib/svg.hpp"

TEST_CASE("Svg produces valid XML structure and elements", "[svg]") {
    turtlelib::Svg svg;
    
    SECTION("Empty SVG has correct header/footer") {
        std::string result = svg.write();
        CHECK_THAT(result, Catch::Matchers::StartsWith("<?xml"));
        CHECK_THAT(result, Catch::Matchers::EndsWith("</svg>"));
        CHECK(result.find("viewBox=\"0 0 816 1056\"") != std::string::npos);
    }

    SECTION("Drawing a point creates a circle at correct coordinates") {
        // Point at (0,0) in turtlelib should be at (408, 528) in SVG
        svg.draw(turtlelib::Point2D{0.0, 0.0}, "purple");
        std::string result = svg.write();
        CHECK(result.find("<circle cx=\"408\" cy=\"528\"") != std::string::npos);
        CHECK(result.find("fill=\"purple\"") != std::string::npos);
    }

    SECTION("Drawing a coordinate frame adds text and axes") {
        svg.draw(turtlelib::Transform2D{}, "World");
        std::string result = svg.write();
        // Should find the label
        CHECK(result.find("{World}") != std::string::npos);
        // Should find red and green lines for axes
        CHECK(result.find("stroke=\"red\"") != std::string::npos);
        CHECK(result.find("stroke=\"green\"") != std::string::npos);
    }
}