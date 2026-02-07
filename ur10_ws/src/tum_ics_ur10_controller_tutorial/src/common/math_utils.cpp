#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>
#include <cmath>

namespace tum_ics_ur10_controller_tutorial
{
namespace math_utils
{

Eigen::Vector3d positionError(
  const Eigen::Vector3d& x_current,
  const Eigen::Vector3d& x_desired)
{
  return x_desired - x_current;
}

Eigen::Vector3d orientationErrorAngleAxis(
  const Eigen::Matrix3d& R_current,
  const Eigen::Matrix3d& R_desired)
{
  // Compute error rotation: R_error = R_desired * R_current^T
  Eigen::Matrix3d R_error = R_desired * R_current.transpose();

  // Convert to angle-axis
  Eigen::AngleAxisd angle_axis(R_error);

  // Return angle * axis as 3D vector
  return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix<double, 6, 1> poseError(
  const Eigen::Affine3d& current,
  const Eigen::Affine3d& desired)
{
  Eigen::Matrix<double, 6, 1> error;

  // Position error (top 3 elements)
  error.head<3>() = positionError(
    current.translation(),
    desired.translation()
  );

  // Orientation error (bottom 3 elements)
  error.tail<3>() = orientationErrorAngleAxis(
    current.rotation(),
    desired.rotation()
  );

  return error;
}

Eigen::MatrixXd pseudoInverse(
  const Eigen::MatrixXd& J,
  double damping)
{
  // Use SVD for robust pseudo-inverse
  // J^+ = V * Sigma^+ * U^T
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    J,
    Eigen::ComputeThinU | Eigen::ComputeThinV
  );

  const Eigen::VectorXd& singular_values = svd.singularValues();
  Eigen::VectorXd singular_values_inv(singular_values.size());

  // Invert singular values with damping
  for (int i = 0; i < singular_values.size(); ++i)
  {
    if (singular_values(i) > damping)
    {
      singular_values_inv(i) = 1.0 / singular_values(i);
    }
    else
    {
      // Damped inverse for small singular values
      singular_values_inv(i) = singular_values(i) /
                               (singular_values(i) * singular_values(i) + damping * damping);
    }
  }

  return svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();
}

Eigen::VectorXd clampMagnitude(
  const Eigen::VectorXd& v,
  double max_mag)
{
  double mag = v.norm();
  if (mag > max_mag)
  {
    return v * (max_mag / mag);
  }
  return v;
}

} // namespace math_utils
} // namespace tum_ics_ur10_controller_tutorial
