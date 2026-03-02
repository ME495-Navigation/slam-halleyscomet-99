/// \file
/// \brief Implementation of the Extended Kalman Filter for SLAM.
#include <cmath>
#include <stdexcept>
#include "turtlelib/ekf.hpp"

namespace turtlelib
{

Ekf::Ekf(size_t max_landmarks) : max_lms(max_landmarks)
{
    const auto state_dim = 3 + 2 * max_lms;
    state = Eigen::VectorXd::Zero(state_dim);

    // Initialize covariance: Robot pose is known (0), landmarks are unknown (infinity)
    cov = Eigen::MatrixXd::Zero(state_dim, state_dim);
    for (size_t i = 3; i < state_dim; ++i) {
        cov(i, i) = 1e6; 
    }

    // Process noise Q - tuned for stable simulation performance
    Q = Eigen::MatrixXd::Identity(3, 3) * 1e-3;

    // Measurement noise R - uncertainty in the Cartesian x,y relative measurements
    R = Eigen::MatrixXd::Identity(2, 2) * 1e-2;

    I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    initialized_landmarks.assign(max_lms, false);
}

void Ekf::predict(Transform2D twist)
{
    const double dtheta = twist.rotation();
    const double dx = twist.translation().x;
    const double old_theta = state(0);
    
    // 1. Update State Estimate using the Non-linear motion model g(x, u)
    if (std::abs(dtheta) < 1e-6) {
        state(1) += dx * std::cos(old_theta);
        state(2) += dx * std::sin(old_theta);
    } else {
        state(0) = normalize_angle(state(0) + dtheta);
        state(1) += (dx / dtheta) * (std::sin(old_theta + dtheta) - std::sin(old_theta));
        state(2) += (dx / dtheta) * (std::cos(old_theta) - std::cos(old_theta + dtheta));
    }

    // 2. Propagate Covariance Sigma = G*Sigma*G^T + Q
    // G is the Jacobian of the motion model w.r.t the state
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(state.size(), state.size());
    if (std::abs(dtheta) < 1e-6) {
        G(1, 0) = -dx * std::sin(old_theta);
        G(2, 0) = dx * std::cos(old_theta);
    } else {
        G(1, 0) = (dx / dtheta) * (std::cos(old_theta + dtheta) - std::cos(old_theta));
        G(2, 0) = (dx / dtheta) * (std::sin(old_theta + dtheta) - std::sin(old_theta));
    }

    cov = G * cov * G.transpose();
    cov.block<3, 3>(0, 0) += Q;
}

void Ekf::update(size_t id, double x, double y)
{
    if (id >= max_lms) return;

    const double robot_theta = state(0);
    const double robot_x = state(1);
    const double robot_y = state(2);

    // 1. Landmark Initialization (Seen for the first time)
    if (!initialized_landmarks.at(id)) {
        // Transform relative sensor coordinates (x,y) to global map coordinates
        state(3 + 2 * id) = robot_x + x * std::cos(robot_theta) - y * std::sin(robot_theta);
        state(4 + 2 * id) = robot_y + x * std::sin(robot_theta) + y * std::cos(robot_theta);
        initialized_landmarks.at(id) = true;
        return; 
    }

    // 2. Calculate Innovation (Measurement Residual)
    const double mx = state(3 + 2 * id);
    const double my = state(4 + 2 * id);
    const double dx = mx - robot_x;
    const double dy = my - robot_y;

    // Predicted Cartesian measurement h(state) relative to robot
    Eigen::Vector2d z_pred;
    z_pred << dx * std::cos(robot_theta) + dy * std::sin(robot_theta),
             -dx * std::sin(robot_theta) + dy * std::cos(robot_theta);
    
    // Observed Cartesian measurement
    Eigen::Vector2d z_actual(x, y);
    Eigen::Vector2d innovation = z_actual - z_pred;

    // 3. Calculate Jacobian H of the measurement model
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state.size());
    const double cos_t = std::cos(robot_theta);
    const double sin_t = std::sin(robot_theta);

    // H w.r.t robot pose (theta, x, y)
    H(0, 0) = -dx * sin_t + dy * cos_t;
    H(0, 1) = -cos_t;
    H(0, 2) = -sin_t;
    H(1, 0) = -dx * cos_t - dy * sin_t;
    H(1, 1) = sin_t;
    H(1, 2) = -cos_t;

    // H w.r.t landmark coordinates (mx, my)
    H(0, 3 + 2 * id) = cos_t;
    H(0, 4 + 2 * id) = sin_t;
    H(1, 3 + 2 * id) = -sin_t;
    H(1, 4 + 2 * id) = cos_t;

    // 4. Kalman Gain calculation
    Eigen::MatrixXd S = H * cov * H.transpose() + R;
    Eigen::MatrixXd K = cov * H.transpose() * S.inverse();

    // 5. Update state and covariance
    state = state + K * innovation;
    state(0) = normalize_angle(state(0)); // Keep orientation within [-pi, pi]
    cov = (I - K * H) * cov;
}

Transform2D Ekf::estimate_pose() const
{
    return Transform2D({state(1), state(2)}, state(0));
}

Eigen::Vector2d Ekf::estimate_landmark(size_t id) const
{
    return {state(3 + 2 * id), state(4 + 2 * id)};
}

} // namespace turtlelib