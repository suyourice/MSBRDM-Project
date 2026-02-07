#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_pdg_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_error.h>
#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>
#include <ros/ros.h>

namespace tum_ics_ur10_controller_tutorial
{

CartesianPDGController::CartesianPDGController(double weight, const QString& name)
  : ControllerBase(weight, name),
    max_velocity_(0.5),
    max_torque_(50.0),
    damping_factor_(0.01)
{
  p_desired_.setZero();
  R_desired_.setIdentity();
  xdot_desired_.setZero();
  pose_error_.setZero();
}

void CartesianPDGController::setDesiredPose(const Eigen::Affine3d& pose)
{
  p_desired_ = pose.translation();
  R_desired_ = pose.rotation();
}

void CartesianPDGController::setDesiredPosition(const Eigen::Vector3d& position)
{
  p_desired_ = position;
}

void CartesianPDGController::setDesiredOrientation(const Eigen::Matrix3d& rotation)
{
  R_desired_ = rotation;
}

void CartesianPDGController::setDesiredVelocity(
  const Eigen::Matrix<double, 6, 1>& velocity)
{
  xdot_desired_ = velocity;
}

Eigen::Affine3d CartesianPDGController::getDesiredPose() const
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() = p_desired_;
  pose.linear() = R_desired_;
  return pose;
}

bool CartesianPDGController::isAtTarget(double pos_tol, double ori_tol) const
{
  if (pose_error_.size() == 0)
    return false;

  double pos_error = pose_error_.head<3>().norm();
  double ori_error = pose_error_.tail<3>().norm();

  return (pos_error < pos_tol) && (ori_error < ori_tol);
}

bool CartesianPDGController::init()
{
  ROS_INFO_STREAM("CartesianPDGController::init()");

  int dof = robot_->getDOF();

  // Allocate gain matrices
  Kp_pos_.setIdentity();
  Kp_ori_.setIdentity();
  Kd_q_ = Eigen::MatrixXd::Zero(dof, dof);

  // Read parameters from ROS
  std::string ns = "~cartesian_pdg_controller";

  // Position gains
  std::vector<double> kp_pos_vec;
  if (nh_.getParam(ns + "/Kp_pos", kp_pos_vec) && kp_pos_vec.size() == 3)
  {
    Kp_pos_ = Eigen::Vector3d(kp_pos_vec.data()).asDiagonal();
  }
  else
  {
    ROS_WARN_STREAM("Kp_pos not found. Using default: 100.0");
    Kp_pos_ *= 100.0;
  }

  // Orientation gains
  std::vector<double> kp_ori_vec;
  if (nh_.getParam(ns + "/Kp_ori", kp_ori_vec) && kp_ori_vec.size() == 3)
  {
    Kp_ori_ = Eigen::Vector3d(kp_ori_vec.data()).asDiagonal();
  }
  else
  {
    ROS_WARN_STREAM("Kp_ori not found. Using default: 50.0");
    Kp_ori_ *= 50.0;
  }

  // Joint damping
  std::vector<double> kd_q_vec;
  if (nh_.getParam(ns + "/Kd_q", kd_q_vec) && kd_q_vec.size() == static_cast<size_t>(dof))
  {
    for (int i = 0; i < dof; ++i)
      Kd_q_(i, i) = kd_q_vec[i];
  }
  else
  {
    ROS_WARN_STREAM("Kd_q not found. Using default: 10.0");
    Kd_q_.setIdentity();
    Kd_q_ *= 10.0;
  }

  // Limits
  nh_.param(ns + "/max_velocity", max_velocity_, 0.5);
  nh_.param(ns + "/max_torque", max_torque_, 50.0);
  nh_.param(ns + "/damping_factor", damping_factor_, 0.01);

  // Initialize desired pose to current pose
  Eigen::Affine3d current_pose = robot_->getEndEffectorPose();
  p_desired_ = current_pose.translation();
  R_desired_ = current_pose.rotation();
  xdot_desired_.setZero();

  ROS_INFO_STREAM("CartesianPDGController initialized");
  ROS_INFO_STREAM("Kp_pos diagonal: " << Kp_pos_.diagonal().transpose());
  ROS_INFO_STREAM("Kp_ori diagonal: " << Kp_ori_.diagonal().transpose());
  ROS_INFO_STREAM("Kd_q diagonal: " << Kd_q_.diagonal().transpose());

  return true;
}

bool CartesianPDGController::start()
{
  ROS_INFO_STREAM("CartesianPDGController::start()");

  // Reset error
  pose_error_.setZero();

  return true;
}

Eigen::VectorXd CartesianPDGController::update(
  const RobotTime& time,
  const JointState& state)
{
  const Eigen::VectorXd& q = state.q;
  const Eigen::VectorXd& qp = state.qp;

  // Get current pose and Jacobian
  Eigen::Affine3d x_current = robot_->getEndEffectorPose();
  Eigen::MatrixXd J = robot_->getJacobian();  // 6xN Jacobian

  // Compute pose error
  Eigen::Vector3d e_pos = cartesian_error::computePositionError(
    x_current.translation(), p_desired_);
  Eigen::Vector3d e_ori = cartesian_error::computeOrientationError(
    x_current.rotation(), R_desired_);

  // Store for monitoring
  pose_error_.head<3>() = e_pos;
  pose_error_.tail<3>() = e_ori;

  // Compute desired Cartesian velocity with feedback
  // ẋ_d_total = ẋ_d + Kp·e
  Eigen::Matrix<double, 6, 1> xdot_d_total = xdot_desired_;
  xdot_d_total.head<3>() += Kp_pos_ * e_pos;
  xdot_d_total.tail<3>() += Kp_ori_ * e_ori;

  // Compute pseudo-inverse of Jacobian
  Eigen::MatrixXd J_pinv = math_utils::pseudoInverse(J, damping_factor_);

  // Compute reference joint velocity
  // q̇_r = J⁺·ẋ_d_total
  Eigen::VectorXd qdot_ref = J_pinv * xdot_d_total;

  // Clamp reference velocity
  qdot_ref = math_utils::clampMagnitude(qdot_ref, max_velocity_);

  // PDG control law
  // τ = -Kd·(q̇ - q̇_r) + Y_r
  // Y_r = 0 for now (no gravity compensation)
  Eigen::VectorXd tau = -Kd_q_ * (qp - qdot_ref);

  // Clamp torque
  tau = math_utils::clampMagnitude(tau, max_torque_);

  return tau;
}

bool CartesianPDGController::stop()
{
  ROS_INFO_STREAM("CartesianPDGController::stop()");
  return true;
}

} // namespace tum_ics_ur10_controller_tutorial
