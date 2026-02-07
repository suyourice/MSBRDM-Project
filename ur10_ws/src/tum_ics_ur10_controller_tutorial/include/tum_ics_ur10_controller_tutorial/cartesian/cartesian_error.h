#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tum_ics_ur10_controller_tutorial
{
namespace cartesian_error
{

/**
 * @brief Compute 6D Cartesian pose error
 *
 * Computes error between current and desired pose:
 * - Position error (3D): e_p = p_d - p
 * - Orientation error (3D): angle-axis representation
 *
 * @param current Current end-effector pose
 * @param desired Desired end-effector pose
 * @return 6D error vector [position_error; orientation_error]
 */
Eigen::Matrix<double, 6, 1> computePoseError(
  const Eigen::Affine3d& current,
  const Eigen::Affine3d& desired
);

/**
 * @brief Compute position error only
 *
 * @param p_current Current position
 * @param p_desired Desired position
 * @return 3D position error (p_d - p)
 */
Eigen::Vector3d computePositionError(
  const Eigen::Vector3d& p_current,
  const Eigen::Vector3d& p_desired
);

/**
 * @brief Compute orientation error using angle-axis
 *
 * Converts rotation error to angle-axis representation:
 *   R_error = R_d * R^T
 *   e_o = angle * axis
 *
 * @param R_current Current orientation
 * @param R_desired Desired orientation
 * @return 3D orientation error vector
 */
Eigen::Vector3d computeOrientationError(
  const Eigen::Matrix3d& R_current,
  const Eigen::Matrix3d& R_desired
);

} // namespace cartesian_error
} // namespace tum_ics_ur10_controller_tutorial
