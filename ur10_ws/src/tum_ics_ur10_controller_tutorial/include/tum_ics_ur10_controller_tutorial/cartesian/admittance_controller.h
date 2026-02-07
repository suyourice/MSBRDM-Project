#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur10_controller_tutorial/sensors/ft_sensor_interface.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Admittance controller for compliant motion
 *
 * Implements admittance dynamics in XY plane:
 *   M·Δẍ + D·Δẋ = F_ext
 *   x_d = x_nominal + Δx
 *
 * Used for force-compliant insertion tasks.
 * Only affects XY plane; Z-axis remains position-controlled.
 */
class AdmittanceController : public ControllerBase
{
public:
  AdmittanceController(double weight = 1.0,
                       const QString& name = "AdmittanceController");

  virtual ~AdmittanceController() = default;

  /**
   * @brief Set nominal trajectory (reference)
   * @param pose Nominal desired pose (without compliance)
   */
  void setNominalPose(const Eigen::Affine3d& pose);

  /**
   * @brief Set nominal position only
   * @param position Nominal position
   */
  void setNominalPosition(const Eigen::Vector3d& position);

  /**
   * @brief Get current compliant pose (nominal + admittance displacement)
   */
  Eigen::Affine3d getCompliantPose() const;

  /**
   * @brief Get admittance displacement
   * @return XY displacement due to force compliance
   */
  Eigen::Vector2d getAdmittanceDisplacement() const;

  /**
   * @brief Reset admittance state (zero displacement and velocity)
   */
  void reset();

protected:
  bool init() override;
  bool start() override;
  Eigen::VectorXd update(const RobotTime& time, const JointState& state) override;
  bool stop() override;

private:
  // Admittance dynamics integration
  void integrateAdmittance(double dt, const Eigen::Vector2d& F_ext);

  // F/T sensor interface
  FTSensorInterface ft_sensor_;

  // Admittance parameters (XY plane only)
  Eigen::Matrix2d M_adm_;  // Virtual mass
  Eigen::Matrix2d D_adm_;  // Damping

  // Admittance state
  Eigen::Vector2d x_adm_;      // Admittance displacement (XY)
  Eigen::Vector2d xdot_adm_;   // Admittance velocity (XY)

  // Nominal trajectory
  Eigen::Vector3d p_nominal_;  // Nominal position
  Eigen::Matrix3d R_nominal_;  // Nominal orientation

  // Cartesian gains (for compliance + position control)
  Eigen::Matrix3d Kp_pos_;
  Eigen::Matrix3d Kp_ori_;
  Eigen::MatrixXd Kd_q_;

  // ROS
  ros::NodeHandle nh_;
  ros::Time last_time_;

  // Limits
  double max_displacement_;  // Maximum admittance displacement (m)
  double max_velocity_;
  double max_torque_;
  double damping_factor_;
};

} // namespace tum_ics_ur10_controller_tutorial
