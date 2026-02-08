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
  JointPDController(const QString& name = "JointPDController", double weight = 1.0);

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
  Eigen::MatrixXd Kp_;  // Proportional gain
  Eigen::MatrixXd Kd_;  // Derivative gain

  // Target position
  Eigen::VectorXd q_desired_;

  // Current error (for monitoring)
  Eigen::VectorXd q_error_;

  // Stored robot configurations
  tum_ics_ur_robot_lli::JointState q_init_;
  tum_ics_ur_robot_lli::JointState q_home_;
  tum_ics_ur_robot_lli::JointState q_park_;

  // ROS node handle
  ros::NodeHandle nh_;

  // Control limits
  double max_torque_;
};

} // namespace tum_ics_ur10_controller_tutorial
