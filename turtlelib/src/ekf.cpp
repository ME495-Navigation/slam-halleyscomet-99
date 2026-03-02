#include <cmath>
#include <stdexcept>
#include "turtlelib/ekf.hpp"

namespace turtlelib
{

Ekf::Ekf(size_t max_landmarks) : max_lms(max_landmarks)
{
    const auto state_dim = 3 + 2 * max_lms;
    
    // Initialize state vector with zeros
    state = Eigen::VectorXd::Zero(state_dim);

    // Initialize covariance matrix
    // Robot pose (first 3x3) is known perfectly at start (zeros)
    // Landmarks (rest) are unknown (infinite uncertainty)
    cov = Eigen::MatrixXd::Zero(state_dim, state_dim);
    for (size_t i = 3; i < state_dim; ++i) {
        cov(i, i) = 1e6; 
    }

    // Process noise (Q) - Uncertainty in movement
    Q = Eigen::MatrixXd::Zero(3, 3);
    // These values may need tuning based on your simulation noise
    Q.diagonal() << 1e-3, 1e-3, 1e-3;

    // Measurement noise (R) - Uncertainty in sensor readings
    R = Eigen::MatrixXd::Identity(2, 2) * 1e-2;

    I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    initialized_landmarks.assign(max_lms, false);
}

void Ekf::predict(Transform2D twist)
{
    const double dtheta = twist.rotation();
    const double dx = twist.translation().x;
    
    // --- 1. Update State Estimate (Non-linear motion model) ---
    const double old_theta = state(0);
    
    if (std::abs(dtheta) < 1e-6) {
        state(1) += dx * std::cos(old_theta);
        state(2) += dx * std::sin(old_theta);
    } else {
        // Differential drive model update
        state(0) += dtheta;
        state(1) += (dx / dtheta) * (std::sin(old_theta + dtheta) - std::sin(old_theta));
        state(2) += (dx / dtheta) * (std::cos(old_theta) - std::cos(old_theta + dtheta));
    }

    // --- 2. Propagate Covariance ---
    // Calculate G (Jacobian of motion model w.r.t state)
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(state.size(), state.size());
    if (std::abs(dtheta) < 1e-6) {
        G(1, 0) = -dx * std::sin(old_theta);
        G(2, 0) = dx * std::cos(old_theta);
    } else {
        G(1, 0) = (dx / dtheta) * (std::cos(old_theta + dtheta) - std::cos(old_theta));
        G(2, 0) = (dx / dtheta) * (std::sin(old_theta + dtheta) - std::sin(old_theta));
    }

    // Sigma = G * Sigma * G^T + Q_bar
    // Note: Q is only added to the robot pose dimensions (3x3)
    cov = G * cov * G.transpose();
    cov.block<3, 3>(0, 0) += Q;
}

void Ekf::update(size_t id, double x, double y)
{
    if (id >= max_lms) {
        throw std::out_of_range("Landmark ID exceeds max_landmarks");
    }

    const double robot_theta = state(0);
    const double robot_x = state(1);
    const double robot_y = state(2);

    // --- 1. Initialization (if seen for the first time) ---
    if (!initialized_landmarks.at(id)) {
        // Convert relative measurement to map-frame absolute coordinates
        state(3 + 2 * id) = robot_x + x * std::cos(robot_theta) - y * std::sin(robot_theta);
        state(4 + 2 * id) = robot_y + x * std::sin(robot_theta) + y * std::cos(robot_theta);
        initialized_landmarks.at(id) = true;
        return; 
    }

    // --- 2. Calculate Innovation ---
    const double mx = state(3 + 2 * id);
    const double my = state(4 + 2 * id);
    const double dx = mx - robot_x;
    const double dy = my - robot_y;
    const double d2 = dx * dx + dy * dy;
    const double d = std::sqrt(d2);

    // Predicted measurement h(state)
    // h(state) = [range; bearing]
    Eigen::Vector2d z_pred;
    z_pred << d, std::atan2(dy, dx) - robot_theta;
    
    // Actual measurement (convert Cartesian input to polar for EKF update)
    Eigen::Vector2d z_actual;
    z_actual << std::sqrt(x * x + y * y), std::atan2(y, x);

    // Innovation y = z - h(state)
    Eigen::Vector2d innovation = z_actual - z_pred;
    // Normalize angle difference to [-pi, pi]
    while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
    while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;

    // --- 3. Calculate Jacobian H ---
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state.size());
    
    // Jacobian of range w.r.t robot pose
    H(0, 0) = 0.0;
    H(0, 1) = -dx / d;
    H(0, 2) = -dy / d;
    // Jacobian of bearing w.r.t robot pose
    H(1, 0) = -1.0;
    H(1, 1) = dy / d2;
    H(1, 2) = -dx / d2;

    // Jacobian w.r.t landmark coordinates
    H(0, 3 + 2 * id) = dx / d;
    H(0, 4 + 2 * id) = dy / d;
    H(1, 3 + 2 * id) = -dy / d2;
    H(1, 4 + 2 * id) = dx / d2;

    // --- 4. Kalman Gain and Correction ---
    // S = H * Sigma * H^T + R
    Eigen::MatrixXd S = H * cov * H.transpose() + R;
    // K = Sigma * H^T * S^-1
    Eigen::MatrixXd K = cov * H.transpose() * S.inverse();

    // Update state and covariance
    state = state + K * innovation;
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