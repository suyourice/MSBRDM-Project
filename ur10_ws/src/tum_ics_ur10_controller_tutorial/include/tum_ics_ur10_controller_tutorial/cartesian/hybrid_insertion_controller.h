#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur10_controller_tutorial/sensors/ft_sensor_interface.h>
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
  HybridInsertionController(double weight = 1.0,
                            const QString& name = "HybridInsertionController");

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

protected:
  bool init() override;
  bool start() override;
  Eigen::VectorXd update(const RobotTime& time, const JointState& state) override;
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
  ros::Time start_time_;
  ros::Time last_time_;

  // ROS
  ros::NodeHandle nh_;

  // Limits
  double max_displacement_;
  double max_velocity_;
  double max_torque_;
  double damping_factor_;
};

} // namespace tum_ics_ur10_controller_tutorial
