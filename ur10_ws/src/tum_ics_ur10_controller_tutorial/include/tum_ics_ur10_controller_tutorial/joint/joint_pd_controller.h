#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Joint space PD controller
 *
 * Implements simple PD control in joint space:
 *   τ = -Kp·(q - q_d) - Kd·q̇
 *
 * Used for:
 * - Moving to home/park positions
 * - Waypoint navigation in joint space
 */
class JointPDController : public ControllerBase
{
public:
  JointPDController(double weight = 1.0, const QString& name = "JointPDController");

  virtual ~JointPDController() = default;

  /**
   * @brief Set desired joint position
   * @param q_desired Target joint configuration
   */
  void setDesiredPosition(const Eigen::VectorXd& q_desired);

  /**
   * @brief Get current desired position
   */
  Eigen::VectorXd getDesiredPosition() const { return q_desired_; }

  /**
   * @brief Check if controller has reached target
   * @param tolerance Position error tolerance (rad)
   */
  bool isAtTarget(double tolerance = 0.05) const;

protected:
  bool init() override;
  bool start() override;
  Eigen::VectorXd update(const RobotTime& time, const JointState& state) override;
  bool stop() override;

private:
  // Controller gains
  Eigen::MatrixXd Kp_;  // Proportional gain
  Eigen::MatrixXd Kd_;  // Derivative gain

  // Target position
  Eigen::VectorXd q_desired_;

  // Current error (for monitoring)
  Eigen::VectorXd q_error_;

  // ROS node handle
  ros::NodeHandle nh_;

  // Control limits
  double max_torque_;
};

} // namespace tum_ics_ur10_controller_tutorial
