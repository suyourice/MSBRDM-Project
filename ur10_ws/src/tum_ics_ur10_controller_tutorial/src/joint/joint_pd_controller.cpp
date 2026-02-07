#include <tum_ics_ur10_controller_tutorial/joint/joint_pd_controller.h>
#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>
#include <ros/ros.h>

namespace tum_ics_ur10_controller_tutorial
{

JointPDController::JointPDController(const QString& name, double weight)
  : ControllerBase(name,
                   tum_ics_ur_robot_lli::RobotControllers::STANDARD_TYPE,
                   tum_ics_ur_robot_lli::RobotControllers::JOINT_SPACE,
                   weight),
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

  // DOF will be determined after setQHome() is called
  // For now, use UR10's 6 DOF as default
  const int dof = 6;

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

  // Note: q_desired_ will be set later (after setQHome() is called)
  // For now, initialize to zeros

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

Tum::VectorDOFd JointPDController::update(
  const tum_ics_ur_robot_lli::RobotTime& time,
  const tum_ics_ur_robot_lli::JointState& state)
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

// Implement additional pure virtuals from Controller
void JointPDController::setQInit(const tum_ics_ur_robot_lli::JointState& qinit)
{
  q_init_ = qinit;
  ROS_INFO_STREAM("JointPDController::setQInit() - stored " << q_init_.q.size() << " joints");
}

void JointPDController::setQHome(const tum_ics_ur_robot_lli::JointState& qhome)
{
  q_home_ = qhome;
  // Initialize desired position to home position
  q_desired_ = q_home_.q;
  ROS_INFO_STREAM("JointPDController::setQHome() - stored " << q_home_.q.size() << " joints");
  ROS_INFO_STREAM("Home position [rad]: " << q_home_.q.transpose());
}

void JointPDController::setQPark(const tum_ics_ur_robot_lli::JointState& qpark)
{
  q_park_ = qpark;
  ROS_INFO_STREAM("JointPDController::setQPark() - stored " << q_park_.q.size() << " joints");
}

} // namespace tum_ics_ur10_controller_tutorial
