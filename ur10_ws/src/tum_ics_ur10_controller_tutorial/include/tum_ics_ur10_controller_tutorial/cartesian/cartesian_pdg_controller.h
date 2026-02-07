#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Cartesian space PDG controller
 *
 * Implements operational space control with velocity reference:
 *   q̇_r = J⁺(ẋ_d + Kp·(x_d - x))
 *   τ = -Kd·(q̇ - q̇_r) + Y_r
 *
 * where:
 *   J⁺ = pseudo-inverse of Jacobian
 *   Y_r = gravity compensation (optional, set to 0 for now)
 *
 * Features:
 * - Position control (3D)
 * - Orientation control (angle-axis, 3D)
 * - Velocity reference tracking
 */
class CartesianPDGController : public ControllerBase
{
public:
  CartesianPDGController(double weight = 1.0,
                         const QString& name = "CartesianPDGController");

  virtual ~CartesianPDGController() = default;

  /**
   * @brief Set desired pose
   * @param pose Desired end-effector pose
   */
  void setDesiredPose(const Eigen::Affine3d& pose);

  /**
   * @brief Set desired position only
   * @param position Desired position (orientation unchanged)
   */
  void setDesiredPosition(const Eigen::Vector3d& position);

  /**
   * @brief Set desired orientation only
   * @param rotation Desired rotation matrix (position unchanged)
   */
  void setDesiredOrientation(const Eigen::Matrix3d& rotation);

  /**
   * @brief Set desired Cartesian velocity
   * @param velocity 6D velocity [linear; angular]
   */
  void setDesiredVelocity(const Eigen::Matrix<double, 6, 1>& velocity);

  /**
   * @brief Get current desired pose
   */
  Eigen::Affine3d getDesiredPose() const;

  /**
   * @brief Check if reached target
   * @param pos_tol Position tolerance (m)
   * @param ori_tol Orientation tolerance (rad)
   */
  bool isAtTarget(double pos_tol = 0.01, double ori_tol = 0.05) const;

protected:
  bool init() override;
  bool start() override;
  Eigen::VectorXd update(const RobotTime& time, const JointState& state) override;
  bool stop() override;

private:
  // Controller gains
  Eigen::Matrix3d Kp_pos_;  // Position gain
  Eigen::Matrix3d Kp_ori_;  // Orientation gain
  Eigen::MatrixXd Kd_q_;    // Joint velocity damping

  // Desired state
  Eigen::Vector3d p_desired_;     // Desired position
  Eigen::Matrix3d R_desired_;     // Desired orientation
  Eigen::Matrix<double, 6, 1> xdot_desired_;  // Desired velocity

  // Current error (for monitoring)
  Eigen::Matrix<double, 6, 1> pose_error_;

  // ROS
  ros::NodeHandle nh_;

  // Limits
  double max_velocity_;
  double max_torque_;
  double damping_factor_;  // For pseudo-inverse
};

} // namespace tum_ics_ur10_controller_tutorial
