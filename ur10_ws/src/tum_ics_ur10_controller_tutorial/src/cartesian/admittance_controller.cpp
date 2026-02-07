#include <tum_ics_ur10_controller_tutorial/cartesian/admittance_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_error.h>
#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>

namespace tum_ics_ur10_controller_tutorial
{

AdmittanceController::AdmittanceController(double weight, const QString& name)
  : ControllerBase(weight, name),
    max_displacement_(0.02),
    max_velocity_(0.5),
    max_torque_(50.0),
    damping_factor_(0.01)
{
  M_adm_.setIdentity();
  D_adm_.setIdentity();
  x_adm_.setZero();
  xdot_adm_.setZero();
  p_nominal_.setZero();
  R_nominal_.setIdentity();
}

void AdmittanceController::setNominalPose(const Eigen::Affine3d& pose)
{
  p_nominal_ = pose.translation();
  R_nominal_ = pose.rotation();
}

void AdmittanceController::setNominalPosition(const Eigen::Vector3d& position)
{
  p_nominal_ = position;
}

Eigen::Affine3d AdmittanceController::getCompliantPose() const
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() = p_nominal_;
  pose.translation().x() += x_adm_(0);  // Add XY compliance
  pose.translation().y() += x_adm_(1);
  pose.linear() = R_nominal_;
  return pose;
}

Eigen::Vector2d AdmittanceController::getAdmittanceDisplacement() const
{
  return x_adm_;
}

void AdmittanceController::reset()
{
  x_adm_.setZero();
  xdot_adm_.setZero();
  ROS_INFO("Admittance state reset");
}

bool AdmittanceController::init()
{
  ROS_INFO_STREAM("AdmittanceController::init()");

  int dof = robot_->getDOF();

  // Allocate matrices
  Kp_pos_.setIdentity();
  Kp_ori_.setIdentity();
  Kd_q_ = Eigen::MatrixXd::Zero(dof, dof);

  // Read parameters
  std::string ns = "~admittance_controller";

  // Admittance parameters (XY only)
  std::vector<double> mass_vec, damping_vec;
  if (nh_.getParam(ns + "/mass_xy", mass_vec) && mass_vec.size() == 2)
  {
    M_adm_(0, 0) = mass_vec[0];
    M_adm_(1, 1) = mass_vec[1];
  }
  else
  {
    ROS_WARN("mass_xy not found. Using default: 0.5 kg");
    M_adm_ *= 0.5;
  }

  if (nh_.getParam(ns + "/damping_xy", damping_vec) && damping_vec.size() == 2)
  {
    D_adm_(0, 0) = damping_vec[0];
    D_adm_(1, 1) = damping_vec[1];
  }
  else
  {
    ROS_WARN("damping_xy not found. Using default: 50.0 Ns/m");
    D_adm_ *= 50.0;
  }

  // Cartesian gains
  std::vector<double> kp_pos_vec;
  if (nh_.getParam(ns + "/Kp_pos", kp_pos_vec) && kp_pos_vec.size() == 3)
  {
    Kp_pos_ = Eigen::Vector3d(kp_pos_vec.data()).asDiagonal();
  }
  else
  {
    Kp_pos_ *= 100.0;
  }

  std::vector<double> kp_ori_vec;
  if (nh_.getParam(ns + "/Kp_ori", kp_ori_vec) && kp_ori_vec.size() == 3)
  {
    Kp_ori_ = Eigen::Vector3d(kp_ori_vec.data()).asDiagonal();
  }
  else
  {
    Kp_ori_ *= 50.0;
  }

  std::vector<double> kd_q_vec;
  if (nh_.getParam(ns + "/Kd_q", kd_q_vec) && kd_q_vec.size() == static_cast<size_t>(dof))
  {
    for (int i = 0; i < dof; ++i)
      Kd_q_(i, i) = kd_q_vec[i];
  }
  else
  {
    Kd_q_.setIdentity();
    Kd_q_ *= 10.0;
  }

  // Limits
  nh_.param(ns + "/max_displacement", max_displacement_, 0.02);
  nh_.param(ns + "/max_velocity", max_velocity_, 0.5);
  nh_.param(ns + "/max_torque", max_torque_, 50.0);
  nh_.param(ns + "/damping_factor", damping_factor_, 0.01);

  // Initialize F/T sensor
  if (!ft_sensor_.init(nh_))
  {
    ROS_ERROR("Failed to initialize F/T sensor");
    return false;
  }

  // Initialize nominal pose to current
  Eigen::Affine3d current_pose = robot_->getEndEffectorPose();
  p_nominal_ = current_pose.translation();
  R_nominal_ = current_pose.rotation();

  ROS_INFO_STREAM("AdmittanceController initialized");
  ROS_INFO_STREAM("Mass (XY): " << M_adm_.diagonal().transpose());
  ROS_INFO_STREAM("Damping (XY): " << D_adm_.diagonal().transpose());

  return true;
}

bool AdmittanceController::start()
{
  ROS_INFO_STREAM("AdmittanceController::start()");

  reset();
  last_time_ = robot_->getTime();

  return true;
}

Eigen::VectorXd AdmittanceController::update(
  const RobotTime& time,
  const JointState& state)
{
  // Compute dt
  double dt = (time - last_time_).toSec();
  last_time_ = time;

  if (dt <= 0.0 || dt > 0.1)
  {
    // Invalid dt, skip update
    return Eigen::VectorXd::Zero(state.q.size());
  }

  // Get external force (XY only)
  Eigen::Vector3d F_measured = ft_sensor_.getForce();
  Eigen::Vector2d F_ext;
  F_ext(0) = F_measured(0);
  F_ext(1) = F_measured(1);

  // Integrate admittance dynamics
  integrateAdmittance(dt, F_ext);

  // Get compliant target pose
  Eigen::Affine3d p_target = getCompliantPose();

  // Get current pose and Jacobian
  Eigen::Affine3d x_current = robot_->getEndEffectorPose();
  Eigen::MatrixXd J = robot_->getJacobian();

  // Compute pose error
  Eigen::Vector3d e_pos = cartesian_error::computePositionError(
    x_current.translation(), p_target.translation());
  Eigen::Vector3d e_ori = cartesian_error::computeOrientationError(
    x_current.rotation(), p_target.rotation());

  // Desired Cartesian velocity
  Eigen::Matrix<double, 6, 1> xdot_d;
  xdot_d.head<3>() = Kp_pos_ * e_pos;
  xdot_d.tail<3>() = Kp_ori_ * e_ori;

  // Pseudo-inverse
  Eigen::MatrixXd J_pinv = math_utils::pseudoInverse(J, damping_factor_);

  // Reference joint velocity
  Eigen::VectorXd qdot_ref = J_pinv * xdot_d;
  qdot_ref = math_utils::clampMagnitude(qdot_ref, max_velocity_);

  // Control law
  Eigen::VectorXd tau = -Kd_q_ * (state.qp - qdot_ref);
  tau = math_utils::clampMagnitude(tau, max_torque_);

  return tau;
}

bool AdmittanceController::stop()
{
  ROS_INFO_STREAM("AdmittanceController::stop()");
  return true;
}

void AdmittanceController::integrateAdmittance(double dt, const Eigen::Vector2d& F_ext)
{
  // Admittance dynamics (2nd order): M·ẍ + D·ẋ = F_ext
  // Rearrange: ẍ = M^-1 (F_ext - D·ẋ)

  Eigen::Matrix2d M_inv = M_adm_.inverse();
  Eigen::Vector2d xddot_adm = M_inv * (F_ext - D_adm_ * xdot_adm_);

  // Integrate velocity: ẋ += ẍ·dt
  xdot_adm_ += xddot_adm * dt;

  // Integrate position: x += ẋ·dt
  x_adm_ += xdot_adm_ * dt;

  // Clamp displacement
  double magnitude = x_adm_.norm();
  if (magnitude > max_displacement_)
  {
    x_adm_ = x_adm_ * (max_displacement_ / magnitude);
    xdot_adm_.setZero();  // Stop at boundary
  }
}

} // namespace tum_ics_ur10_controller_tutorial
