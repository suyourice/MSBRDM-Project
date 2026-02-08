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
    damping_factor_(0.01),
    max_lin_velocity_(0.1),
    max_ori_velocity_(0.5),
    use_task_pd_(true),
    task_pos_scale_(1.0),
    task_ori_scale_(1.0)
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

  ROS_INFO_STREAM("CartesianPDG: setDesiredPose called");
  ROS_INFO_STREAM("  New p_desired: " << p_desired_.transpose());
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
  Kp_q_ = Eigen::MatrixXd::Zero(dof, dof);
  use_joint_p_ = false;
  use_gravity_comp_ = true;
  dynamic_model_ = nullptr;
  q_ref_ = Eigen::VectorXd::Zero(dof);
  q_ref_initialized_ = false;
  last_time_ = -1.0;

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

  // Optional joint-space proportional term (integrated velocity reference)
  std::vector<double> kp_q_vec;
  if (pnh.getParam(ns + "/Kp_q", kp_q_vec) && kp_q_vec.size() == static_cast<size_t>(dof))
  {
    for (int i = 0; i < dof; ++i)
      Kp_q_(i, i) = kp_q_vec[i];
    use_joint_p_ = true;
    ROS_INFO_STREAM("Kp_q loaded from YAML (joint P term enabled)");
  }
  pnh.param(ns + "/use_joint_p", use_joint_p_, use_joint_p_);

  // Limits
  pnh.param(ns + "/max_velocity", max_velocity_, 0.5);
  pnh.param(ns + "/max_torque", max_torque_, 50.0);
  pnh.param(ns + "/damping_factor", damping_factor_, 0.01);
  pnh.param(ns + "/max_lin_velocity", max_lin_velocity_, 0.1);  // Limit translation speed to preserve direction
  pnh.param(ns + "/max_ori_velocity", max_ori_velocity_, 0.5);  // Limit orientation velocity to prevent coupling issues
  pnh.param(ns + "/task_pd", use_task_pd_, true);
  pnh.param(ns + "/task_pos_scale", task_pos_scale_, 1.0);
  pnh.param(ns + "/task_ori_scale", task_ori_scale_, 1.0);
  pnh.param(ns + "/use_gravity_comp", use_gravity_comp_, true);

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

    // Create dynamic model for gravity compensation (optional)
    if (use_gravity_comp_)
    {
      dynamic_model_ = new tum_ics_ur_robot_lli::Robot::DynamicModel(
        QString::fromStdString(config_file_path),
        kinematic_model_);
      if (dynamic_model_->error())
      {
        ROS_WARN_STREAM("DynamicModel initialization failed: "
                        << dynamic_model_->errorString().toStdString());
        delete dynamic_model_;
        dynamic_model_ = nullptr;
      }
      else
      {
        ROS_INFO_STREAM("DynamicModel created successfully (gravity compensation enabled)");
      }
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception creating KinematicModel: " << e.what());
    kinematic_model_ = nullptr;
    if (dynamic_model_)
    {
      delete dynamic_model_;
      dynamic_model_ = nullptr;
    }
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
  if (use_joint_p_)
  {
    ROS_INFO_STREAM("Kp_q diagonal: " << Kp_q_.diagonal().transpose());
  }

  return true;
}

bool CartesianPDGController::start()
{
  ROS_INFO_STREAM("CartesianPDGController::start()");

  // Reset error
  pose_error_.setZero();
  q_ref_initialized_ = false;
  last_time_ = -1.0;

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

  // DEBUG: Check Jacobian size ONCE
  static bool jacobian_checked = false;
  if (!jacobian_checked) {
    ROS_ERROR_STREAM("JACOBIAN SIZE CHECK: " << J.rows() << " x " << J.cols());
    ROS_ERROR_STREAM("J(0,0:2) = " << J(0,0) << ", " << J(0,1) << ", " << J(0,2));
    ROS_ERROR_STREAM("J(2,0:2) = " << J(2,0) << ", " << J(2,1) << ", " << J(2,2));
    jacobian_checked = true;
  }

  // Compute pose error
  Eigen::Vector3d e_pos = cartesian_error::computePositionError(
    x_current.translation(), p_desired_);
  Eigen::Vector3d e_ori = cartesian_error::computeOrientationError(
    x_current.rotation(), R_desired_);

  // Debug output (throttled)
  static int debug_counter = 0;
  if (debug_counter++ % 500 == 0)  // Every 1 second (at 500Hz)
  {
    ROS_INFO_STREAM("CartesianPDG Debug:");
    ROS_INFO_STREAM("  Current pos: " << x_current.translation().transpose());
    ROS_INFO_STREAM("  Desired pos: " << p_desired_.transpose());
    ROS_INFO_STREAM("  Pos error:   " << e_pos.transpose());
    ROS_INFO_STREAM("  Ori error:   " << e_ori.transpose());

    // More detailed debug - check control flow
    Eigen::Matrix<double, 6, 1> xdot_check = xdot_desired_;
    xdot_check.head<3>() += Kp_pos_ * e_pos;
    ROS_INFO_STREAM("  xdot (before ori): " << xdot_check.head<3>().transpose());
    ROS_INFO_STREAM("  Joint velocities (first 3): " << qp.head<3>().transpose());
  }

  // Store for monitoring
  pose_error_.head<3>() = e_pos;
  pose_error_.tail<3>() = e_ori;

  // PDG (resolved-rate) control:
  // q̇_r = J⁺(ẋ_d + Kp·e), τ = -Kd·(q̇ - q̇_r) [+ Kp_q·(q_ref - q)]

  // Compute desired Cartesian velocity with feedback
  Eigen::Matrix<double, 6, 1> xdot_d_total = xdot_desired_;
  Eigen::Vector3d lin_velocity = xdot_desired_.head<3>() + Kp_pos_ * e_pos;
  double lin_vel_norm = lin_velocity.norm();
  if (lin_vel_norm > max_lin_velocity_)
  {
    lin_velocity *= (max_lin_velocity_ / lin_vel_norm);
  }
  xdot_d_total.head<3>() = lin_velocity;

  Eigen::Vector3d ori_velocity = Kp_ori_ * e_ori;
  double ori_vel_norm = ori_velocity.norm();
  if (ori_vel_norm > max_ori_velocity_)
  {
    ori_velocity *= (max_ori_velocity_ / ori_vel_norm);
  }
  xdot_d_total.tail<3>() += ori_velocity;

  Eigen::MatrixXd J_pinv = math_utils::pseudoInverse(J, damping_factor_);
  Eigen::VectorXd qdot_ref = J_pinv * xdot_d_total;
  qdot_ref = math_utils::clampAbs(qdot_ref, max_velocity_);

  // Integrate reference velocity to build a joint position reference
  double t_now = time.tD();
  if (!q_ref_initialized_)
  {
    q_ref_ = q;
    last_time_ = t_now;
    q_ref_initialized_ = true;
  }
  double dt = t_now - last_time_;
  if (dt > 0.0 && dt < 0.1)
  {
    q_ref_ += qdot_ref * dt;
  }
  last_time_ = t_now;

  Eigen::VectorXd tau = -Kd_q_ * (qp - qdot_ref);
  if (use_joint_p_)
  {
    tau += Kp_q_ * (q_ref_ - q);
  }

  if (use_task_pd_)
  {
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.head<3>() = task_pos_scale_ * (Kp_pos_ * e_pos);
    wrench.tail<3>() = task_ori_scale_ * (Kp_ori_ * e_ori);
    tau += J.transpose() * wrench;
  }

  // Gravity compensation
  if (use_gravity_comp_ && dynamic_model_)
  {
    dynamic_model_->update(time, state);
    tau += dynamic_model_->G();
  }

  if (debug_counter % 500 == 0)
  {
    ROS_INFO_STREAM("  qdot_ref (first 3): " << qdot_ref.head<3>().transpose());
    ROS_INFO_STREAM("  tau (first 3): " << tau.head<3>().transpose());
    ROS_INFO_STREAM("  qp (first 3): " << qp.head<3>().transpose());
  }

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
  if (dynamic_model_)
  {
    delete dynamic_model_;
    dynamic_model_ = nullptr;
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
