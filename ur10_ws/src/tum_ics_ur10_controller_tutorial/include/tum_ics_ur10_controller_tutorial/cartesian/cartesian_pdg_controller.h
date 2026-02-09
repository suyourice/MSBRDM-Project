#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur10_controller_tutorial/sensors/ft_sensor_interface.h>
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
  /**
   * @brief Operating modes for CartesianPDGController
   *
   * CRITICAL: Admittance is NOT a separate controller!
   * It's an internal mode that modifies p_desired based on force feedback.
   */
  enum class Mode {
    TRACKING,              ///< Normal position/orientation tracking
    INSERTION_ADMITTANCE   ///< Peg-in-hole with circular search + XY force compliance
  };

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
   * @brief Sync reference to current EE pose (for smooth transitions)
   * CRITICAL: Call this before changing targets to prevent jumps
   */
  void syncToCurrentEE();

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

  /**
   * @brief Set operating mode
   * @param mode TRACKING or INSERTION_ADMITTANCE
   *
   * CRITICAL: This allows mode switching WITHOUT changing controller instance!
   * FSM should call this instead of switching between different controllers.
   */
  void setMode(Mode mode);

  /**
   * @brief Get current operating mode
   */
  Mode getMode() const { return mode_; }

  /**
   * @brief Set insertion center position (for INSERTION_ADMITTANCE mode)
   * @param center Starting position for circular trajectory
   */
  void setInsertionCenter(const Eigen::Vector3d& center);

  /**
   * @brief Set insertion parameters (for INSERTION_ADMITTANCE mode)
   * @param radius Circular search radius (m)
   * @param omega Angular velocity (rad/s)
   * @param v_descent Z-axis descent velocity (m/s)
   * @param z_target Target insertion depth (m)
   */
  void setInsertionParams(double radius, double omega, double v_descent, double z_target);

  /**
   * @brief Set admittance parameters (for INSERTION_ADMITTANCE mode)
   * @param M Virtual mass matrix (2x2, XY only)
   * @param D Damping matrix (2x2, XY only)
   */
  void setAdmittanceParams(const Eigen::Matrix2d& M, const Eigen::Matrix2d& D);

  /**
   * @brief Check if insertion is complete
   * @return True if z_depth >= target AND force_drop detected
   */
  bool isInsertionComplete() const { return insertion_success_; }

  /**
   * @brief Get current insertion depth
   * @return Z-axis distance inserted (m)
   */
  double getCurrentDepth() const;

  /**
   * @brief Reset insertion state (for retry)
   */
  void resetInsertionState();

  // FSM lifecycle hooks
  void onEnterState(const tum_ics_ur_robot_lli::RobotTime& time,
                    const tum_ics_ur_robot_lli::JointState& state) override;
  void onExitState() override;

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
  // Operating mode
  Mode mode_;

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

  // F/T sensor (for INSERTION_ADMITTANCE mode)
  FTSensorInterface ft_sensor_;

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

  // ========================================================================
  // INSERTION_ADMITTANCE Mode State (Mode::INSERTION_ADMITTANCE only)
  // ========================================================================

  // Insertion trajectory parameters
  Eigen::Vector3d insertion_center_;   ///< Center of circular motion
  double insertion_start_time_;        ///< Time when insertion started (for trajectory)
  double insertion_radius_;            ///< Circular search radius (m)
  double insertion_omega_;             ///< Angular velocity (rad/s)
  double insertion_v_descent_;         ///< Z-axis descent velocity (m/s)
  double insertion_z_target_;          ///< Target insertion depth (m)

  // Success detection
  double force_initial_z_;             ///< Initial Z-axis force (recorded at mode entry)
  double force_drop_threshold_;        ///< Force drop threshold for success (N)
  bool insertion_success_;             ///< Success flag

  // Admittance dynamics (XY plane only)
  Eigen::Matrix2d M_adm_;              ///< Virtual mass matrix [2x2]
  Eigen::Matrix2d D_adm_;              ///< Damping matrix [2x2]
  Eigen::Vector2d x_adm_;              ///< Admittance displacement [XY]
  Eigen::Vector2d xd_adm_;             ///< Admittance velocity [XY]
  double max_compliance_disp_;         ///< Safety limit on admittance displacement (m)
};

} // namespace tum_ics_ur10_controller_tutorial
