#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_error.h>

namespace tum_ics_ur10_controller_tutorial
{
namespace cartesian_error
{

Eigen::Vector3d computePositionError(
  const Eigen::Vector3d& p_current,
  const Eigen::Vector3d& p_desired)
{
  return p_desired - p_current;
}

Eigen::Vector3d computeOrientationError(
  const Eigen::Matrix3d& R_current,
  const Eigen::Matrix3d& R_desired)
{
  // Compute rotation error: R_error = R_d * R^T
  Eigen::Matrix3d R_error = R_desired * R_current.transpose();

  // Convert to angle-axis representation
  Eigen::AngleAxisd angle_axis(R_error);

  // Return as 3D vector: angle * axis
  return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix<double, 6, 1> computePoseError(
  const Eigen::Affine3d& current,
  const Eigen::Affine3d& desired)
{
  Eigen::Matrix<double, 6, 1> error;

  // Position error (first 3 elements)
  error.head<3>() = computePositionError(
    current.translation(),
    desired.translation()
  );

  // Orientation error (last 3 elements)
  error.tail<3>() = computeOrientationError(
    current.rotation(),
    desired.rotation()
  );

  return error;
}

} // namespace cartesian_error
} // namespace tum_ics_ur10_controller_tutorial
