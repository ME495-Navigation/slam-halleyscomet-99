#include <iostream>
#include <cmath>
#include <string>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        std::string unit_str = "";
        char c;
        if (is.peek() == '<') {
            is >> c >> tw.omega;
            if (std::isalpha(is.peek())) { is >> unit_str; }
            is >> c >> tw.x >> c >> tw.y >> c;
        } else {
            is >> tw.omega;
            if (std::isalpha(is.peek())) { is >> unit_str; }
            is >> tw.x >> tw.y;
        }
        if (!unit_str.empty() && unit_str[0] == 'd') {
            tw.omega = deg2rad(tw.omega);
        }
        return is;
    }

    Transform2D::Transform2D() : x_(0.0), y_(0.0), theta_(0.0) {}

    Transform2D::Transform2D(Vector2D trans) : x_(trans.x), y_(trans.y), theta_(0.0) {}

    Transform2D::Transform2D(double radians) : x_(0.0), y_(0.0), theta_(radians) {}

    Transform2D::Transform2D(Vector2D trans, double radians)
        : x_(trans.x), y_(trans.y), theta_(radians) {}

    Point2D Transform2D::operator()(Point2D p) const
    {
        return {p.x * std::cos(theta_) - p.y * std::sin(theta_) + x_,
                p.x * std::sin(theta_) + p.y * std::cos(theta_) + y_};
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        return {v.x * std::cos(theta_) - v.y * std::sin(theta_),
                v.x * std::sin(theta_) + v.y * std::cos(theta_)};
    }

    Twist2D Transform2D::operator()(Twist2D v) const
    {
        // Using the Adjoint transformation for SE(2)
        return {v.omega,
                y_ * v.omega + std::cos(theta_) * v.x - std::sin(theta_) * v.y,
                -x_ * v.omega + std::sin(theta_) * v.x + std::cos(theta_) * v.y};
    }

    Transform2D Transform2D::inv() const
    {
        // T^-1 = [R^T, -R^T * p]
        double inv_theta = -theta_;
        double inv_x = -(x_ * std::cos(theta_) + y_ * std::sin(theta_));
        double inv_y = -(-x_ * std::sin(theta_) + y_ * std::cos(theta_));
        return Transform2D({inv_x, inv_y}, inv_theta);
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        double new_x = rhs.x_ * std::cos(theta_) - rhs.y_ * std::sin(theta_) + x_;
        double new_y = rhs.x_ * std::sin(theta_) + rhs.y_ * std::cos(theta_) + y_;
        theta_ += rhs.theta_;
        x_ = new_x;
        y_ = new_y;
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return {x_, y_};
    }

    double Transform2D::rotation() const
    {
        return theta_;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        double theta = 0.0, dx = 0.0, dy = 0.0;
        std::string unit_str = "";
        char c;

        // Peek to see if we have the bracket format "{"
        if (is.peek() == '{') {
            is >> c; // consume '{'
            is >> theta;
            
            // Peek for unit (e.g., "deg")
            is >> std::ws;
            if (std::isalpha(is.peek())) {
                is >> unit_str;
            }

            // Handle the comma after the angle/unit
            if (is.peek() == ',') { is >> c; } 

            is >> dx;
            if (is.peek() == ',') { is >> c; } // handle comma
            is >> dy;
            
            if (is.peek() == '}') { is >> c; } // consume '}'
        } else {
            // Handle "theta dx dy" format
            is >> theta;
            is >> std::ws;
            if (std::isalpha(is.peek())) {
                is >> unit_str;
            }
            is >> dx >> dy;
        }

        if (!unit_str.empty() && unit_str[0] == 'd') {
            theta = deg2rad(theta);
        }
        
        tf = Transform2D({dx, dy}, theta);
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs *= rhs;
    }
}