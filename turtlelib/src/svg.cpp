#include "turtlelib/svg.hpp"
#include <sstream>
#include <cmath>

namespace turtlelib
{
Svg::Svg() {}

void Svg::draw(Point2D p, std::string color)
{
        // Map turtlelib (x, y) to SVG (ox + x*96, oy - y*96)
  double sx = ox + p.x * 96.0;
  double sy = oy - p.y * 96.0;
  std::stringstream ss;
  ss << "<circle cx=\"" << sx << "\" cy=\"" << sy << "\" r=\"3\" fill=\"" << color << "\" />";
  elements.push_back(ss.str());
}

void Svg::draw(Point2D tail, Vector2D v, std::string color)
{
  Point2D head = tail + v;
  double x1 = ox + tail.x * 96.0, y1 = oy - tail.y * 96.0;
  double x2 = ox + head.x * 96.0, y2 = oy - head.y * 96.0;

  std::stringstream ss;
  ss       << "<line x1=\"" << x1 << "\" y1=\"" << y1 << "\" x2=\"" << x2 << "\" y2=\"" << y2
           << "\" stroke=\"" << color << "\" stroke-width=\"2\" marker-end=\"url(#arrowhead)\" />";
  elements.push_back(ss.str());
}

void Svg::draw(Transform2D tf, std::string label)
{
        // Use the operator() to transform origin and unit axes
  Point2D origin = tf(Point2D{0.0, 0.0});
  Vector2D x_axis = tf(Vector2D{0.5, 0.0});       // 0.5 meter axis
  Vector2D y_axis = tf(Vector2D{0.0, 0.5});

        // Draw axes lines
  draw(origin, x_axis, "red");
  draw(origin, y_axis, "green");

  double sx = ox + origin.x * 96.0;
  double sy = oy - origin.y * 96.0;
  std::stringstream ss;
  ss << "<text x=\"" << sx << "\" y=\"" << sy << "\">{" << label << "}</text>";
  elements.push_back(ss.str());
}

std::string Svg::write() const
{
  std::stringstream ss;
  ss << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
  ss <<
    "<svg width=\"8.5in\" height=\"11in\" viewBox=\"0 0 816 1056\" xmlns=\"http://www.w3.org/2000/svg\">\n";

        // Marker definition for arrows
  ss << "<defs>\n";
  ss <<
    "  <marker id=\"arrowhead\" markerWidth=\"10\" markerHeight=\"7\" refX=\"0\" refY=\"3.5\" orient=\"auto\">\n";
  ss << "    <polygon points=\"0 0, 10 3.5, 0 7\" fill=\"black\" />\n";
  ss << "  </marker>\n";
  ss << "</defs>\n";

  for (const auto & e : elements) {
    ss << "  " << e << "\n";
  }

  ss << "</svg>";
  return ss.str();
}
}
