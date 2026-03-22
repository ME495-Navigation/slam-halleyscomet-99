/// \file
/// \brief Implementation of 2D geometry primitives, distance metrics, and circle fitting.

#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{
std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  return os << "(" << p.x << " " << p.y << ")";
}

std::istream & operator>>(std::istream & is, Point2D & p)
{
  auto c = is.peek();
  if (c == '(') {
    is.get();
    is >> p.x;
    is >> std::ws;
    if (is.peek() == ',') {
      is.get();
    }
    is >> p.y;
    is >> std::ws;
    if (is.peek() == ')') {
      is.get();
    }
  } else {
    is >> p.x >> p.y;
  }
  return is;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  return os << "[" << v.x << " " << v.y << "]";
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  auto c = is.peek();
  if (c == '[') {
    is.get();
    is >> v.x;
    is >> std::ws;
    if (is.peek() == ',') {
      is.get();
    }
    is >> v.y;
    is >> std::ws;
    if (is.peek() == ']') {
      is.get();
    }
  } else {
    is >> v.x >> v.y;
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

Vector2D & Vector2D::operator+=(const Vector2D & rhs)
{
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
{
  return lhs += rhs;
}

Vector2D & Vector2D::operator-=(const Vector2D & rhs)
{
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}

Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
{
  return lhs -= rhs;
}

Vector2D & Vector2D::operator*=(const double s)
{
  x *= s;
  y *= s;
  return *this;
}

Vector2D operator*(Vector2D v, const double s)
{
  return v *= s;
}

Vector2D operator*(const double s, Vector2D v)
{
  return v *= s;
}

double dot(Vector2D v1, Vector2D v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

double magnitude(Vector2D v)
{
  return std::sqrt(dot(v, v));
}

double distance(Point2D p1, Point2D p2)
{
  return magnitude(p1 - p2);
}

double angle(Vector2D v1, Vector2D v2)
{
  const auto m1 = magnitude(v1);
  const auto m2 = magnitude(v2);
  if (almost_equal(m1, 0.0) || almost_equal(m2, 0.0)) {
    return 0.0;
  }
  return std::acos(dot(v1, v2) / (m1 * m2));
}

Vector2D normalize(Vector2D in)
{
  const auto mag = magnitude(in);
  if (almost_equal(mag, 0.0)) {
    throw std::invalid_argument("Cannot normalize a zero vector.");
  }
  return {in.x / mag, in.y / mag};
}

Circle fit_circle(const std::vector<Point2D> & cluster)
{
  const auto n = static_cast<double>(cluster.size());
  if (n < 3.0) {
    throw std::invalid_argument("Circle fitting requires at least 3 points.");
  }

  // 1. Compute the centroid (x_bar, y_bar)
  double x_sum = 0.0, y_sum = 0.0;
  for (const auto & p : cluster) {
    x_sum += p.x;
    y_sum += p.y;
  }
  const double x_bar = x_sum / n;
  const double y_bar = y_sum / n;

  // 2. Shift coordinates so that the centroid is at the origin.
  // 3. Compute z_bar = (1/n) * sum(xi^2 + yi^2)
  double z_sum = 0.0;
  std::vector<double> xi, yi, zi;
  for (const auto & p : cluster) {
    const double x_shifted = p.x - x_bar;
    const double y_shifted = p.y - y_bar;
    const double z = x_shifted * x_shifted + y_shifted * y_shifted;
    xi.push_back(x_shifted);
    yi.push_back(y_shifted);
    zi.push_back(z);
    z_sum += z;
  }
  const double z_bar = z_sum / n;

  // 4. Form the data matrix Z (n x 4)
  Eigen::MatrixXd Z(cluster.size(), 4);
  for (size_t i = 0; i < cluster.size(); ++i) {
    Z(i, 0) = zi.at(i);
    Z(i, 1) = xi.at(i);
    Z(i, 2) = yi.at(i);
    Z(i, 3) = 1.0;
  }

  // 5. Form the constraint matrix H_inv (Pratt Constraint)
  // Reference: A. Al-Sharadqah and N. Chernov, "Error Analysis for Circle Fitting Algorithms",
  // Electronic Journal of Statistics, 2009.
  Eigen::Matrix4d H_inv = Eigen::Matrix4d::Zero();
  H_inv(0, 3) = 0.5;
  H_inv(1, 1) = 1.0;
  H_inv(2, 2) = 1.0;
  H_inv(3, 0) = 0.5;
  H_inv(3, 3) = -2.0 * z_bar;

  // 6. Singular Value Decomposition of Z
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeFullV);
  const Eigen::VectorXd sigma = svd.singularValues();
  const Eigen::MatrixXd V = svd.matrixV();

  Eigen::VectorXd A;

  // 7. If the smallest singular value is near zero, A is the last column of V
  if (sigma(3) < 1e-12) {
    A = V.col(3);
  } else {
    // 8. Form matrix Y = V * Sigma * V^T
    Eigen::Matrix4d Sigma = Eigen::Matrix4d::Zero();
    for (int i = 0; i < 4; ++i) {Sigma(i, i) = sigma(i);}
    const Eigen::Matrix4d Y = V * Sigma * V.transpose();

    // 9. Form matrix Q = Y * H_inv * Y
    const Eigen::Matrix4d Q_mat = Y * H_inv * Y;

    // 10. Compute eigenvectors and eigenvalues of Q
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(Q_mat);
    const Eigen::VectorXd & eigenvalues = es.eigenvalues();
    const Eigen::MatrixXd & eigenvectors = es.eigenvectors();

    // ############################ Begin_Citation [1] ############################
    // Find the index of the smallest *positive* eigenvalue.
    // SelfAdjointEigenSolver returns eigenvalues in ascending order, so we
    // iterate from the smallest and take the first one above the numerical
    // zero threshold. This is the correct Pratt constraint solution.
    int min_idx = -1;
    double min_val = std::numeric_limits<double>::max();
    for (int i = 0; i < 4; ++i) {
      if (eigenvalues(i) > 1e-12 && eigenvalues(i) < min_val) {
        min_val = eigenvalues(i);
        min_idx = i;
      }
    }
    // ############################ End_Citation [1] ############################

    if (min_idx == -1) {
      throw std::runtime_error("Pratt fit: no positive eigenvalue found — degenerate cluster.");
    }

    // 11. Solve for A_star and back-substitute A = Y^{-1} * A_star
    const Eigen::VectorXd A_star = eigenvectors.col(min_idx);
    A = Y.ldlt().solve(A_star);
  }

  // 12. Extract circle parameters from the algebraic form A = [a, b1, b2, c]^T.
  // The implicit circle equation is: a*(x^2+y^2) + b1*x + b2*y + c = 0.
  const double a  = A(0);
  const double b1 = A(1);
  const double b2 = A(2);
  const double c  = A(3);

  // Centre and radius in the shifted (centroid-origin) frame, then translate back.
  const double x_c = -b1 / (2.0 * a);
  const double y_c = -b2 / (2.0 * a);
  const double R_fitted = std::sqrt((b1 * b1 + b2 * b2 - 4.0 * a * c) / (4.0 * a * a));

  return {{x_c + x_bar, y_c + y_bar}, R_fitted};
}

} // namespace turtlelib