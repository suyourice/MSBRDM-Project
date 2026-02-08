#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_pdg_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_error.h>
#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace tum_ics_ur10_controller_tutorial
{

CartesianPDGController::CartesianPDGController(const QString& name, double weight)
  : ControllerBase(name,
                   tum_ics_ur_robot_lli::RobotControllers::STANDARD_TYPE,
                   tum_ics_ur_robot_lli::RobotControllers::CARTESIAN_SPACE,
                   weight),
    kinematic_model_(nullptr),
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

  // DOF for UR10
  const int dof = 6;

  // Allocate gain matrices
  Kp_pos_.setIdentity();
  Kp_ori_.setIdentity();
  Kd_q_ = Eigen::MatrixXd::Zero(dof, dof);

  // Read parameters from ROS (private namespace)
  ros::NodeHandle pnh("~");
  std::string ns = "cartesian_pdg_controller";

  // Position gains
  std::vector<double> kp_pos_vec;
  if (pnh.getParam(ns + "/Kp_pos", kp_pos_vec) && kp_pos_vec.size() == 3)
  {
    Kp_pos_ = Eigen::Vector3d(kp_pos_vec.data()).asDiagonal();
    ROS_INFO_STREAM("Kp_pos loaded from YAML: " << kp_pos_vec[0] << ", " << kp_pos_vec[1] << ", " << kp_pos_vec[2]);
  }
  else
  {
    ROS_WARN_STREAM("Kp_pos not found. Using default: 30.0");
    Kp_pos_ *= 30.0;
  }

  std::vector<double> kp_ori_vec;
  if (pnh.getParam(ns + "/Kp_ori", kp_ori_vec) && kp_ori_vec.size() == 3)
  {
    Kp_ori_ = Eigen::Vector3d(kp_ori_vec.data()).asDiagonal();
    ROS_INFO_STREAM("Kp_ori loaded from YAML: " << kp_ori_vec[0] << ", " << kp_ori_vec[1] << ", " << kp_ori_vec[2]);
  }
  else
  {
    ROS_WARN_STREAM("Kp_ori not found. Using default: 20.0");
    Kp_ori_ *= 20.0;
  }

  // Joint damping
  std::vector<double> kd_q_vec;
  if (pnh.getParam(ns + "/Kd_q", kd_q_vec) && kd_q_vec.size() == static_cast<size_t>(dof))
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
  pnh.param(ns + "/max_velocity", max_velocity_, 0.5);
  pnh.param(ns + "/max_torque", max_torque_, 50.0);
  pnh.param(ns + "/damping_factor", damping_factor_, 0.01);

  // Get config file path from ROS parameter
  std::string config_file_path;
  if (!pnh.getParam("/ur_config_file", config_file_path))
  {
    // Default path
    config_file_path = ros::package::getPath("tum_ics_ur10_controller_tutorial") +
                       "/launch/configs/configUR10.ini";
    ROS_WARN_STREAM("Config file path not found in ROS params. Using default: "
                    << config_file_path);
  }

  // Create kinematic model
  try
  {
    kinematic_model_ = new tum_ics_ur_robot_lli::Robot::KinematicModel(
      QString::fromStdString(config_file_path),
      "none"  // group name
    );

    if (kinematic_model_->error())
    {
      ROS_ERROR_STREAM("KinematicModel initialization failed: "
                      << kinematic_model_->errorString().toStdString());
      delete kinematic_model_;
      kinematic_model_ = nullptr;
      return false;
    }

    ROS_INFO_STREAM("KinematicModel created successfully");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception creating KinematicModel: " << e.what());
    kinematic_model_ = nullptr;
    return false;
  }

  // Initialize desired pose to zeros (will be set by setQHome or user)
  p_desired_.setZero();
  R_desired_.setIdentity();
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

Tum::VectorDOFd CartesianPDGController::update(
  const tum_ics_ur_robot_lli::RobotTime& time,
  const tum_ics_ur_robot_lli::JointState& state)
{
  const Eigen::VectorXd& q = state.q;
  const Eigen::VectorXd& qp = state.qp;

  // Update kinematic model with current state
  if (kinematic_model_)
  {
    kinematic_model_->update(time, state);
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "KinematicModel not initialized!");
    return Eigen::VectorXd::Zero(q.size());
  }

  // Get current pose and Jacobian from kinematic model
  const Eigen::Affine3d& x_current = kinematic_model_->Tef_0();
  const Eigen::MatrixXd& J = kinematic_model_->Jef_0();  // 6xN Jacobian

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
  qdot_ref = math_utils::clampAbs(qdot_ref, max_velocity_);

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

  // Clean up kinematic model
  if (kinematic_model_)
  {
    delete kinematic_model_;
    kinematic_model_ = nullptr;
  }

  return true;
}

// Implement additional pure virtuals from Controller
void CartesianPDGController::setQInit(const tum_ics_ur_robot_lli::JointState& qinit)
{
  q_init_ = qinit;
  ROS_INFO_STREAM("CartesianPDGController::setQInit() - stored " << q_init_.q.size() << " joints");
}

void CartesianPDGController::setQHome(const tum_ics_ur_robot_lli::JointState& qhome)
{
  q_home_ = qhome;

  // Update kinematic model and set desired pose to home pose
  if (kinematic_model_)
  {
    // Compute FK for home position
    Eigen::Affine3d home_pose = kinematic_model_->Tef_0(q_home_.q);
    p_desired_ = home_pose.translation();
    R_desired_ = home_pose.rotation();

    ROS_INFO_STREAM("CartesianPDGController::setQHome() - stored " << q_home_.q.size() << " joints");
    ROS_INFO_STREAM("Home EE position: " << p_desired_.transpose());
  }
  else
  {
    ROS_WARN_STREAM("CartesianPDGController::setQHome() - KinematicModel not initialized yet");
  }
}

void CartesianPDGController::setQPark(const tum_ics_ur_robot_lli::JointState& qpark)
{
  q_park_ = qpark;
  ROS_INFO_STREAM("CartesianPDGController::setQPark() - stored " << q_park_.q.size() << " joints");
}

} // namespace tum_ics_ur10_controller_tutorial
