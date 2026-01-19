#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD

/// \file
/// \brief Two-dimensional geometric primitives and other mathematical objects

#include <iosfwd>
#include <format>

namespace turtlelib
{
    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief Input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   "(x, y)" or "x y"  (Not including the "").
    /// \param is An istream from which to read
    /// \param p [out] The Point2D object that will store the input
    /// \returns A reference to is. An error flag is set on the stream if the input cannot be parsed.
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    Point2D operator+(const Point2D & tail, const Vector2D & disp);

    /// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   "[x, y]" or "x y" (not including the "")
    /// \param is An istream from which to read
    /// \param v [out] - output vector
    /// \returns a reference to the istream, with any error flags set if
    /// a parsing error occurs
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief Return a unit vector in the direction of v
    /// \param in The vector to normalize
    /// \return The normalized vector.
    /// \throws std::invalid_argument if in is the zero vector
    Vector2D normalize(Vector2D in);
}

/// \cond
/// \brief A Formatter for 2D points. Output is (x, y)
template<class CharT>
struct std::formatter<turtlelib::Point2D, CharT> : std::formatter<double, CharT>
{
    /// \brief Formats the Point2D object

    constexpr auto parse(std::format_parse_context & ctx) {
        return std::formatter<double, CharT>::parse(ctx);
    }
    
    template<typename FormatContext>
    auto format(const turtlelib::Point2D & p, FormatContext & ctx) const
    {
        auto it = ctx.out();
        it = std::format_to(it, "(");
        it = std::formatter<double, CharT>::format(p.x, ctx);
        it = std::format_to(it, ", ");
        it = std::formatter<double, CharT>::format(p.y, ctx);
        return std::format_to(it, ")");
    }
};

/// \brief A Formatter for 2D vectors. Output is [x, y]
template<class CharT>
struct std::formatter<turtlelib::Vector2D, CharT> : std::formatter<double, CharT>
{
    /// \brief Formats the Vector2D object
    template<typename FormatContext>
    auto format(const turtlelib::Vector2D & v, FormatContext & ctx) const
    {
        auto it = ctx.out();
        it = std::format_to(it, "[");
        it = std::formatter<double, CharT>::format(v.x, ctx);
        it = std::format_to(it, ", ");
        it = std::formatter<double, CharT>::format(v.y, ctx);
        return std::format_to(it, "]");
    }
};
/// \endcond

#endif