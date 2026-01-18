#include <iostream>
#include <cmath>
#include <stdexcept>
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        char c = is.peek();
        if (c == '(') {
            is.get(); // consume '('
            is >> p.x;
            is >> std::ws; // skip whitespace
            if (is.peek() == ',') {
                is.get(); // consume ','
            }
            is >> p.y;
            is >> std::ws;
            if (is.peek() == ')') {
                is.get(); // consume ')'
            }
        } else {
            is >> p.x >> p.y;
        }
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {
        return {head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp)
    {
        return {tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        return os << "[" << v.x << ", " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek();
        if (c == '[') {
            is.get(); // consume '['
            is >> v.x;
            is >> std::ws;
            if (is.peek() == ',') {
                is.get(); // consume ','
            }
            is >> v.y;
            is >> std::ws;
            if (is.peek() == ']') {
                is.get(); // consume ']'
            }
        } else {
            is >> v.x >> v.y;
        }
        return is;
    }

    Vector2D normalize(Vector2D in)
    {
        const auto mag = std::sqrt(in.x * in.x + in.y * in.y);
        if (std::abs(mag) < 1e-12) {
            throw std::invalid_argument("Cannot normalize a zero vector.");
        }
        return {in.x / mag, in.y / mag};
    }
}