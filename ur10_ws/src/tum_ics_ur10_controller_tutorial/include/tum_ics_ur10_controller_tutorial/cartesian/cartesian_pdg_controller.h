#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include <tum_ics_ur_robot_lli/Robot/DynamicModel.h>
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
  CartesianPDGController(const QString& name = "CartesianPDGController",
                         double weight = 1.0);

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

  // Public wrappers for FSM to call protected methods
  bool callInit() { return init(); }
  bool callStart() { return start(); }
  Tum::VectorDOFd callUpdate(const tum_ics_ur_robot_lli::RobotTime& time,
                             const tum_ics_ur_robot_lli::JointState& state) {
    return update(time, state);
  }
  bool callStop() { return stop(); }

  // Additional pure virtuals from Controller base (public for FSM access)
  void setQInit(const tum_ics_ur_robot_lli::JointState& qinit) override;
  void setQHome(const tum_ics_ur_robot_lli::JointState& qhome) override;
  void setQPark(const tum_ics_ur_robot_lli::JointState& qpark) override;

protected:
  bool init() override;
  bool start() override;
  Tum::VectorDOFd update(const tum_ics_ur_robot_lli::RobotTime& time,
                         const tum_ics_ur_robot_lli::JointState& state) override;
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

  // Kinematic model
  tum_ics_ur_robot_lli::Robot::KinematicModel* kinematic_model_;
  tum_ics_ur_robot_lli::Robot::DynamicModel* dynamic_model_;
  bool use_gravity_comp_;

  // Stored robot configurations
  tum_ics_ur_robot_lli::JointState q_init_;
  tum_ics_ur_robot_lli::JointState q_home_;
  tum_ics_ur_robot_lli::JointState q_park_;

  // ROS
  ros::NodeHandle nh_;

  // Limits
  double max_velocity_;
  double max_torque_;
  double damping_factor_;  // For pseudo-inverse
  double max_lin_velocity_;  // Cap translational velocity to avoid saturation artifacts
  double max_ori_velocity_;  // Cap orientation velocity to prevent coupling issues

  // Optional task-space PD term (Jacobian transpose)
  bool use_task_pd_;
  double task_pos_scale_;
  double task_ori_scale_;

  // Optional joint-space proportional term (integrated velocity reference)
  bool use_joint_p_;
  Eigen::MatrixXd Kp_q_;
  Eigen::VectorXd q_ref_;
  bool q_ref_initialized_;
  double last_time_;
};

} // namespace tum_ics_ur10_controller_tutorial
