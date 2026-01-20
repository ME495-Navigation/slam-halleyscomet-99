#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD

/// \file
/// \brief Vector graphics export for visualizing 2D geometry.

#include <vector>
#include <string>
#include <iosfwd>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief A class to generate SVG files representing 2D geometry objects.
class Svg
{
public:
        /// \brief Create an empty SVG with default dimensions (8.5x11 inches).
  Svg();

        /// \brief Draw a point.
        /// \param p The point to draw (in turtlelib space).
        /// \param color The stroke/fill color.
  void draw(Point2D p, std::string color = "purple");

        /// \brief Draw a vector.
        /// \param tail The starting point of the vector.
        /// \param v The vector itself.
        /// \param color The stroke color.
  void draw(Point2D tail, Vector2D v, std::string color = "orange");

        /// \brief Draw a coordinate frame.
        /// \param tf The transformation representing the frame's pose.
        /// \param label A text label for the frame.
  void draw(Transform2D tf, std::string label);

        /// \brief Get the SVG content as a string.
        /// \return A string containing the full XML content of the SVG.
  std::string write() const;

private:
        /// \brief Buffered SVG XML elements.
  std::vector<std::string> elements;

        // Constants for conversion (assuming 96 DPI)
  const double width = 816.0;        ///< 8.5 inches * 96 DPI
  const double height = 1056.0;       ///< 11 inches * 96 DPI
  const double ox = 408.0;            ///< Center x in SVG pixels
  const double oy = 528.0;            ///< Center y in SVG pixels
};
}

#endif
