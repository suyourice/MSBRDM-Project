#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur10_controller_tutorial/sensors/ft_sensor_interface.h>
#include <tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Hybrid controller for peg-in-hole insertion
 *
 * Combines three control modes:
 * - Z-axis: Position control (constant descent velocity)
 * - XY-plane: Circular trajectory (spiral search pattern)
 * - XY-plane: Admittance control (force compliance)
 *
 * Control strategy:
 *   x_d.z -= v_insert * dt                    // Descend
 *   x_d.x = x_center + r*cos(ω*t) + Δx_adm   // Circular + compliance
 *   x_d.y = y_center + r*sin(ω*t) + Δy_adm   // Circular + compliance
 *
 * Success detection:
 *   (z_inserted >= z_target) AND (F_z < F_threshold)
 */
class HybridInsertionController : public ControllerBase
{
public:
  HybridInsertionController(const QString& name = "HybridInsertionController",
                            double weight = 1.0);

  virtual ~HybridInsertionController() = default;

  /**
   * @brief Set insertion starting position (center of circular motion)
   * @param position Starting position (above hole)
   */
  void setStartPosition(const Eigen::Vector3d& position);

  /**
   * @brief Set target insertion depth
   * @param depth Target Z depth (m, positive = downward)
   */
  void setTargetDepth(double depth);

  /**
   * @brief Check if insertion succeeded
   */
  bool isInsertionComplete() const;

  /**
   * @brief Get current insertion depth
   */
  double getCurrentDepth() const;

  /**
   * @brief Reset insertion (for retry)
   */
  void reset();

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
  // Update desired position (circular + descent + admittance)
  void updateDesiredPosition(double dt);

  // Integrate admittance dynamics
  void integrateAdmittance(double dt, const Eigen::Vector2d& F_ext);

  // Check insertion success
  bool checkSuccess();

  // F/T sensor
  FTSensorInterface ft_sensor_;

  // Kinematic model
  tum_ics_ur_robot_lli::Robot::KinematicModel* kinematic_model_;

  // Stored robot configurations
  tum_ics_ur_robot_lli::JointState q_init_;
  tum_ics_ur_robot_lli::JointState q_home_;
  tum_ics_ur_robot_lli::JointState q_park_;

  // Circular motion parameters
  double radius_;       // Search radius (m)
  double omega_;        // Angular velocity (rad/s)
  Eigen::Vector3d center_;  // Center position

  // Insertion parameters
  double v_insert_;     // Descent velocity (m/s)
  double z_start_;      // Starting Z position
  double z_target_;     // Target insertion depth

  // Admittance parameters (XY only)
  Eigen::Matrix2d M_adm_;
  Eigen::Matrix2d D_adm_;
  Eigen::Vector2d x_adm_;      // Admittance displacement
  Eigen::Vector2d xdot_adm_;   // Admittance velocity

  // Current desired pose
  Eigen::Vector3d p_desired_;
  Eigen::Matrix3d R_desired_;

  // Success detection
  double force_drop_threshold_;  // Force reduction (N)
  double force_initial_;         // Initial insertion force
  bool insertion_complete_;

  // Cartesian gains
  Eigen::Matrix3d Kp_pos_;
  Eigen::Matrix3d Kp_ori_;
  Eigen::MatrixXd Kd_q_;

  // Time tracking
  tum_ics_ur_robot_lli::RobotTime start_time_;
  tum_ics_ur_robot_lli::RobotTime last_time_;

  // ROS
  ros::NodeHandle nh_;

  // Limits
  double max_displacement_;
  double max_velocity_;
  double max_torque_;
  double damping_factor_;
};

} // namespace tum_ics_ur10_controller_tutorial
