#ifndef TURTLELIB_EKF_HPP_INCLUDE
#define TURTLELIB_EKF_HPP_INCLUDE

#include <Eigen/Dense>
#include <vector>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

/// \brief Feature-based Extended Kalman Filter SLAM
class Ekf
{
public:
    /// \brief Initialize the EKF with initial pose and high uncertainty for landmarks
    /// \param max_landmarks - The maximum number of landmarks the system will track
    explicit Ekf(size_t max_landmarks = 20);

    /// \brief The Prediction Step of EKF SLAM
    /// \param twist - The relative twist (movement) since the last update
    void predict(Transform2D twist);

    /// \brief The Correction Step for a known landmark
    /// \param id - The index/ID of the landmark
    /// \param x - Measured relative x position of landmark
    /// \param y - Measured relative y position of landmark
    void update(size_t id, double x, double y);

    /// \brief Get the current estimated robot pose
    /// \return The pose as a Transform2D
    Transform2D estimate_pose() const;

    /// \brief Get the estimated position of a specific landmark
    /// \param id - Landmark ID
    /// \return Vector containing {x, y}
    Eigen::Vector2d estimate_landmark(size_t id) const;

private:
    // State Vector Size: 3 (pose) + 2 * max_landmarks
    size_t max_lms;
    
    /// \brief State vector: [theta, x, y, mx1, my1, ..., mxn, myn]^T
    Eigen::VectorXd state;

    /// \brief Covariance matrix Sigma
    Eigen::MatrixXd cov;

    /// \brief Process noise Q
    Eigen::MatrixXd Q;

    /// \brief Measurement noise R
    Eigen::MatrixXd R;

    /// \brief Identity matrix for internal calculations
    Eigen::MatrixXd I;

    /// \brief Flag to track if a landmark has been initialized
    std::vector<bool> initialized_landmarks;
};

} // namespace turtlelib

#endif