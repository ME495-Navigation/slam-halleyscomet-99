/// \file
/// \brief Feature-based Extended Kalman Filter SLAM implementation.
#ifndef TURTLELIB_EKF_HPP_INCLUDE
#define TURTLELIB_EKF_HPP_INCLUDE

#include <Eigen/Dense>
#include <vector>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

/// \brief Feature-based Extended Kalman Filter SLAM
/// \details Tracks the robot pose and landmark positions in a joint state vector.
class Ekf
{
public:
    /// \brief Initialize the EKF with initial pose and high uncertainty for landmarks.
    /// \param max_landmarks - The maximum number of landmarks the system will track.
    explicit Ekf(size_t max_landmarks = 20);

    /// \brief The Prediction Step of EKF SLAM.
    /// \details Updates the state estimate using a non-linear motion model and 
    /// propagates the covariance matrix.
    /// \param twist - The relative twist (body frame) since the last update.
    void predict(Transform2D twist);

    /// \brief The Correction Step for a known landmark.
    /// \details Updates the joint state vector and covariance matrix using 
    /// a Cartesian measurement model.
    /// \param id - The known ID of the landmark.
    /// \param x - Measured relative x position of landmark.
    /// \param y - Measured relative y position of landmark.
    void update(size_t id, double x, double y);

    /// \brief Get the current estimated robot pose.
    /// \return The pose as a Transform2D in the map frame.
    Transform2D estimate_pose() const;

    /// \brief Get the estimated position of a specific landmark.
    /// \param id - Landmark ID.
    /// \return Vector containing {x, y} in the map frame.
    Eigen::Vector2d estimate_landmark(size_t id) const;

private:
    size_t max_lms;
    
    /// \brief State vector: [theta, x, y, mx0, my0, ..., mxn, myn]^T
    Eigen::VectorXd state;

    /// \brief Covariance matrix Sigma (Joint robot and landmark uncertainty)
    Eigen::MatrixXd cov;

    /// \brief Process noise Q (Robot motion uncertainty)
    Eigen::MatrixXd Q;

    /// \brief Measurement noise R (Sensor uncertainty)
    Eigen::MatrixXd R;

    /// \brief Identity matrix for internal calculations
    Eigen::MatrixXd I;

    /// \brief Flag to track if a landmark has been initialized in the state vector
    std::vector<bool> initialized_landmarks;
};

} // namespace turtlelib

#endif