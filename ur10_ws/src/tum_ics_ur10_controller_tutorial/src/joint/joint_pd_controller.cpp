#include <tum_ics_ur10_controller_tutorial/joint/joint_pd_controller.h>
#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>
#include <ros/ros.h>

namespace tum_ics_ur10_controller_tutorial
{

JointPDController::JointPDController(double weight, const QString& name)
  : ControllerBase(weight, name),
    max_torque_(50.0)
{
}

void JointPDController::setDesiredPosition(const Eigen::VectorXd& q_desired)
{
  q_desired_ = q_desired;
}

bool JointPDController::isAtTarget(double tolerance) const
{
  if (q_error_.size() == 0)
    return false;

  return q_error_.norm() < tolerance;
}

bool JointPDController::init()
{
  ROS_INFO_STREAM("JointPDController::init()");

  // Get DOF from robot
  int dof = robot_->getDOF();

  // Allocate gain matrices
  Kp_ = Eigen::MatrixXd::Zero(dof, dof);
  Kd_ = Eigen::MatrixXd::Zero(dof, dof);
  q_desired_ = Eigen::VectorXd::Zero(dof);
  q_error_ = Eigen::VectorXd::Zero(dof);

  // Read parameters from ROS parameter server
  std::string ns = "~joint_pd_controller";

  // Read Kp gains
  std::vector<double> kp_vec;
  if (nh_.getParam(ns + "/Kp", kp_vec))
  {
    if (kp_vec.size() == static_cast<size_t>(dof))
    {
      for (int i = 0; i < dof; ++i)
        Kp_(i, i) = kp_vec[i];
    }
    else
    {
      ROS_WARN_STREAM("Kp size mismatch. Expected " << dof << ", got " << kp_vec.size());
      // Use default values
      Kp_.setIdentity();
      Kp_ *= 100.0;
    }
  }
  else
  {
    ROS_WARN_STREAM("Kp not found in parameter server. Using default values.");
    Kp_.setIdentity();
    Kp_ *= 100.0;
  }

  // Read Kd gains
  std::vector<double> kd_vec;
  if (nh_.getParam(ns + "/Kd", kd_vec))
  {
    if (kd_vec.size() == static_cast<size_t>(dof))
    {
      for (int i = 0; i < dof; ++i)
        Kd_(i, i) = kd_vec[i];
    }
    else
    {
      ROS_WARN_STREAM("Kd size mismatch. Expected " << dof << ", got " << kd_vec.size());
      // Use default values
      Kd_.setIdentity();
      Kd_ *= 10.0;
    }
  }
  else
  {
    ROS_WARN_STREAM("Kd not found in parameter server. Using default values.");
    Kd_.setIdentity();
    Kd_ *= 10.0;
  }

  // Read max torque limit
  nh_.param(ns + "/max_torque", max_torque_, 50.0);

  // Initialize desired position to current home position
  q_desired_ = robot_->qHome();

  ROS_INFO_STREAM("JointPDController initialized with DOF=" << dof);
  ROS_INFO_STREAM("Kp diagonal: " << Kp_.diagonal().transpose());
  ROS_INFO_STREAM("Kd diagonal: " << Kd_.diagonal().transpose());
  ROS_INFO_STREAM("Max torque: " << max_torque_);

  return true;
}

bool JointPDController::start()
{
  ROS_INFO_STREAM("JointPDController::start()");

  // Reset error
  q_error_.setZero();

  return true;
}

Eigen::VectorXd JointPDController::update(
  const RobotTime& time,
  const JointState& state)
{
  const Eigen::VectorXd& q = state.q;
  const Eigen::VectorXd& qp = state.qp;

  // Compute position error
  q_error_ = q_desired_ - q;

  // PD control law: τ = -Kp·(q - q_d) - Kd·q̇
  //                   = Kp·(q_d - q) - Kd·q̇
  Eigen::VectorXd tau = Kp_ * q_error_ - Kd_ * qp;

  // Apply torque limits
  tau = math_utils::clampMagnitude(tau, max_torque_);

  return tau;
}

bool JointPDController::stop()
{
  ROS_INFO_STREAM("JointPDController::stop()");
  return true;
}

} // namespace tum_ics_ur10_controller_tutorial
