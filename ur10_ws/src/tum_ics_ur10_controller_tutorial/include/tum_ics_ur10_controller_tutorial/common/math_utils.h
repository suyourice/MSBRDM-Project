#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tum_ics_ur10_controller_tutorial
{
namespace math_utils
{

/**
 * @brief Compute position error
 *
 * Simple Euclidean distance error.
 *
 * @param x_current Current position
 * @param x_desired Desired position
 * @return Position error (x_desired - x_current)
 */
Eigen::Vector3d positionError(
  const Eigen::Vector3d& x_current,
  const Eigen::Vector3d& x_desired
);

/**
 * @brief Compute orientation error using angle-axis representation
 *
 * Computes the orientation error as:
 *   e_o = angle * axis
 * where (angle, axis) is the angle-axis representation of R_error = R_desired * R_current^T
 *
 * This provides a minimal 3D representation of orientation error on SO(3).
 *
 * @param R_current Current orientation (rotation matrix)
 * @param R_desired Desired orientation (rotation matrix)
 * @return Orientation error as angle-axis vector (3D)
 */
Eigen::Vector3d orientationErrorAngleAxis(
  const Eigen::Matrix3d& R_current,
  const Eigen::Matrix3d& R_desired
);

/**
 * @brief Compute 6D pose error (position + orientation)
 *
 * Combines position error and orientation error (angle-axis).
 *
 * @param current Current pose
 * @param desired Desired pose
 * @return 6D error vector [position_error; orientation_error]
 */
Eigen::Matrix<double, 6, 1> poseError(
  const Eigen::Affine3d& current,
  const Eigen::Affine3d& desired
);

/**
 * @brief Compute pseudo-inverse using SVD
 *
 * Computes J^+ with damping for numerical stability.
 *
 * @param J Input matrix (usually Jacobian)
 * @param damping Damping factor for singularity robustness (default 0.01)
 * @return Pseudo-inverse J^+
 */
Eigen::MatrixXd pseudoInverse(
  const Eigen::MatrixXd& J,
  double damping = 0.01
);

/**
 * @brief Clamp vector to maximum magnitude
 *
 * @param v Input vector
 * @param max_mag Maximum magnitude
 * @return Clamped vector
 */
Eigen::VectorXd clampMagnitude(
  const Eigen::VectorXd& v,
  double max_mag
);

} // namespace math_utils
} // namespace tum_ics_ur10_controller_tutorial
