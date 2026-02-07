#pragma once

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

class OperationalSpaceController : public tum_ics_ur_robot_lli::RobotControllers::ControlEffort
{
public:
  enum class Phase
  {
    PARK,
    SAFE,
    LINEAR,
    CIRCLE,
    DONE
  };

  OperationalSpaceController(double weight = 1.0,
                             const QString& name = "OperationalSpaceController");

  virtual ~OperationalSpaceController() = default;

private:
  // LLI lifecycle
  bool init() override;
  bool start() override;
  Eigen::VectorXd update(const RobotTime& time,
                         const JointState& state) override;
  bool stop() override;

private:
  // helpers
  Eigen::Vector3d computeCartesianError(const Eigen::Affine3d& x,
                                        const Eigen::Affine3d& xd);
  Eigen::Vector3d desiredCartesianVelocity(const RobotTime& time);

private:
  Phase phase_;
  RobotTime phase_start_;

  // gains
  Eigen::Matrix3d Kp_x_;
  Eigen::Matrix3d Kd_x_;
  Eigen::MatrixXd Kp_q_;

  // references
  Eigen::VectorXd q_home_;
  Eigen::VectorXd q_safe_;

  // trajectory params
  Eigen::Vector3d circle_center_;
  double radius_;
  double omega_;
};

} // namespace tum_ics_ur10_controller_tutorial
