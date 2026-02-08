#include <tum_ics_ur10_controller_tutorial/cartesian/hybrid_insertion_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_error.h>
#include <tum_ics_ur10_controller_tutorial/common/math_utils.h>
#include <ros/package.h>

namespace tum_ics_ur10_controller_tutorial
{

HybridInsertionController::HybridInsertionController(const QString& name, double weight)
  : ControllerBase(name,
                   tum_ics_ur_robot_lli::RobotControllers::STANDARD_TYPE,
                   tum_ics_ur_robot_lli::RobotControllers::CARTESIAN_SPACE,
                   weight),
    kinematic_model_(nullptr),
    radius_(0.003),
    omega_(0.5),
    v_insert_(0.002),
    z_target_(0.025),
    force_drop_threshold_(3.0),
    force_initial_(0.0),
    insertion_complete_(false),
    max_displacement_(0.02),
    max_velocity_(0.5),
    max_torque_(50.0),
    damping_factor_(0.01)
{
  center_.setZero();
  p_desired_.setZero();
  R_desired_.setIdentity();
  M_adm_.setIdentity();
  D_adm_.setIdentity();
  x_adm_.setZero();
  xdot_adm_.setZero();
}

void HybridInsertionController::setStartPosition(const Eigen::Vector3d& position)
{
  center_ = position;
  z_start_ = position.z();
  p_desired_ = position;
}

void HybridInsertionController::setTargetDepth(double depth)
{
  z_target_ = depth;
}

bool HybridInsertionController::isInsertionComplete() const
{
  return insertion_complete_;
}

double HybridInsertionController::getCurrentDepth() const
{
  return z_start_ - p_desired_.z();
}

void HybridInsertionController::reset()
{
  x_adm_.setZero();
  xdot_adm_.setZero();
  insertion_complete_ = false;
  force_initial_ = 0.0;
  ROS_INFO("Hybrid insertion controller reset");
}

bool HybridInsertionController::init()
{
  ROS_INFO_STREAM("HybridInsertionController::init()");

  const int dof = 6;

  // Allocate matrices
  Kp_pos_.setIdentity();
  Kp_ori_.setIdentity();
  Kd_q_ = Eigen::MatrixXd::Zero(dof, dof);

  // Read parameters (private namespace)
  ros::NodeHandle pnh("~");
  std::string ns = "hybrid_insertion_controller";

  // Circular motion
  pnh.param(ns + "/radius", radius_, 0.003);
  pnh.param(ns + "/omega", omega_, 0.5);

  // Insertion
  pnh.param(ns + "/v_insert", v_insert_, 0.002);
  pnh.param(ns + "/z_target", z_target_, 0.025);

  // Admittance
  std::vector<double> mass_vec, damping_vec;
  if (pnh.getParam(ns + "/mass_xy", mass_vec) && mass_vec.size() == 2)
  {
    M_adm_(0, 0) = mass_vec[0];
    M_adm_(1, 1) = mass_vec[1];
  }
  else
  {
    M_adm_ *= 0.5;
  }

  if (pnh.getParam(ns + "/damping_xy", damping_vec) && damping_vec.size() == 2)
  {
    D_adm_(0, 0) = damping_vec[0];
    D_adm_(1, 1) = damping_vec[1];
  }
  else
  {
    D_adm_ *= 50.0;
  }

  // Success criteria
  pnh.param(ns + "/force_drop_threshold", force_drop_threshold_, 3.0);

  std::vector<double> kp_pos_vec;
  if (pnh.getParam(ns + "/Kp_pos", kp_pos_vec) && kp_pos_vec.size() == 3)
  {
    Kp_pos_ = Eigen::Vector3d(kp_pos_vec.data()).asDiagonal();
    ROS_INFO_STREAM("Kp_pos loaded from YAML: " << kp_pos_vec[0] << ", " << kp_pos_vec[1] << ", " << kp_pos_vec[2]);
  }
  else
  {
    Kp_pos_ *= 30.0;
    ROS_WARN_STREAM("Kp_pos not found. Using default: 30.0");
  }

  std::vector<double> kp_ori_vec;
  if (pnh.getParam(ns + "/Kp_ori", kp_ori_vec) && kp_ori_vec.size() == 3)
  {
    Kp_ori_ = Eigen::Vector3d(kp_ori_vec.data()).asDiagonal();
    ROS_INFO_STREAM("Kp_ori loaded from YAML: " << kp_ori_vec[0] << ", " << kp_ori_vec[1] << ", " << kp_ori_vec[2]);
  }
  else
  {
    Kp_ori_ *= 20.0;
    ROS_WARN_STREAM("Kp_ori not found. Using default: 20.0");
  }

  std::vector<double> kd_q_vec;
  if (pnh.getParam(ns + "/Kd_q", kd_q_vec) && kd_q_vec.size() == static_cast<size_t>(dof))
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
  pnh.param(ns + "/max_displacement", max_displacement_, 0.02);
  pnh.param(ns + "/max_velocity", max_velocity_, 0.5);
  pnh.param(ns + "/max_torque", max_torque_, 50.0);
  pnh.param(ns + "/damping_factor", damping_factor_, 0.01);

  // Initialize F/T sensor
  if (!ft_sensor_.init(pnh))
  {
    ROS_ERROR("Failed to initialize F/T sensor");
    return false;
  }

  // Get config file path
  std::string config_file_path;
  if (!pnh.getParam("/ur_config_file", config_file_path))
  {
    config_file_path = ros::package::getPath("tum_ics_ur10_controller_tutorial") +
                       "/launch/configs/configUR10.ini";
    ROS_WARN_STREAM("Config file path not found. Using default: " << config_file_path);
  }

  // Create kinematic model
  try
  {
    kinematic_model_ = new tum_ics_ur_robot_lli::Robot::KinematicModel(
      QString::fromStdString(config_file_path),
      "none"
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

  // Initialize pose to zeros
  center_.setZero();
  z_start_ = 0.0;
  p_desired_.setZero();
  R_desired_.setIdentity();

  ROS_INFO_STREAM("HybridInsertionController initialized");
  ROS_INFO_STREAM("  Radius: " << radius_ << " m");
  ROS_INFO_STREAM("  Omega: " << omega_ << " rad/s");
  ROS_INFO_STREAM("  v_insert: " << v_insert_ << " m/s");
  ROS_INFO_STREAM("  z_target: " << z_target_ << " m");

  return true;
}

bool HybridInsertionController::start()
{
  ROS_INFO_STREAM("HybridInsertionController::start()");

  reset();
  start_time_ = tum_ics_ur_robot_lli::RobotTime();
  last_time_ = tum_ics_ur_robot_lli::RobotTime();

  // Record initial force
  force_initial_ = ft_sensor_.getForce().z();

  return true;
}

Tum::VectorDOFd HybridInsertionController::update(
  const tum_ics_ur_robot_lli::RobotTime& time,
  const tum_ics_ur_robot_lli::JointState& state)
{
  // Initialize time on first call
  if (last_time_.tD() == 0.0)
  {
    last_time_ = time;
    start_time_ = time;
  }

  // Compute dt
  double dt = (time.tRc() - last_time_.tRc()).toSec();
  last_time_ = time;

  if (dt <= 0.0 || dt > 0.1)
  {
    return Eigen::VectorXd::Zero(state.q.size());
  }

  // Update kinematic model
  if (kinematic_model_)
  {
    kinematic_model_->update(time, state);
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "KinematicModel not initialized!");
    return Eigen::VectorXd::Zero(state.q.size());
  }

  // Check if already complete
  if (insertion_complete_)
  {
    return Eigen::VectorXd::Zero(state.q.size());
  }

  // Update desired position (circular + descent + admittance)
  updateDesiredPosition(dt);

  // Check success
  if (checkSuccess())
  {
    insertion_complete_ = true;
    ROS_INFO("Insertion complete!");
    return Eigen::VectorXd::Zero(state.q.size());
  }

  // Get current pose and Jacobian from kinematic model
  const Eigen::Affine3d& x_current = kinematic_model_->Tef_0();
  const Eigen::MatrixXd& J = kinematic_model_->Jef_0();

  // Compute pose error
  Eigen::Vector3d e_pos = cartesian_error::computePositionError(
    x_current.translation(), p_desired_);
  Eigen::Vector3d e_ori = cartesian_error::computeOrientationError(
    x_current.rotation(), R_desired_);

  // Desired velocity
  Eigen::Matrix<double, 6, 1> xdot_d;
  xdot_d.head<3>() = Kp_pos_ * e_pos;
  xdot_d.tail<3>() = Kp_ori_ * e_ori;

  // Pseudo-inverse
  Eigen::MatrixXd J_pinv = math_utils::pseudoInverse(J, damping_factor_);

  // Reference joint velocity
  Eigen::VectorXd qdot_ref = J_pinv * xdot_d;
  qdot_ref = math_utils::clampAbs(qdot_ref, max_velocity_);

  // Control law
  Eigen::VectorXd tau = -Kd_q_ * (state.qp - qdot_ref);
  tau = math_utils::clampMagnitude(tau, max_torque_);

  return tau;
}

bool HybridInsertionController::stop()
{
  ROS_INFO_STREAM("HybridInsertionController::stop()");

  // Clean up kinematic model
  if (kinematic_model_)
  {
    delete kinematic_model_;
    kinematic_model_ = nullptr;
  }

  return true;
}

void HybridInsertionController::updateDesiredPosition(double dt)
{
  // Time since start
  double t = (last_time_.tRc() - start_time_.tRc()).toSec();

  // 1. Z-axis: Descend at constant velocity
  p_desired_.z() -= v_insert_ * dt;

  // 2. XY-plane: Circular motion
  double x_circle = center_.x() + radius_ * std::cos(omega_ * t);
  double y_circle = center_.y() + radius_ * std::sin(omega_ * t);

  // 3. XY-plane: Admittance (force compliance)
  Eigen::Vector3d F_measured = ft_sensor_.getForce();
  Eigen::Vector2d F_ext;
  F_ext(0) = F_measured(0);
  F_ext(1) = F_measured(1);

  integrateAdmittance(dt, F_ext);

  // 4. Combine: circular + admittance
  p_desired_.x() = x_circle + x_adm_(0);
  p_desired_.y() = y_circle + x_adm_(1);
}

void HybridInsertionController::integrateAdmittance(double dt, const Eigen::Vector2d& F_ext)
{
  // Admittance dynamics: M·ẍ + D·ẋ = F_ext
  Eigen::Matrix2d M_inv = M_adm_.inverse();
  Eigen::Vector2d xddot_adm = M_inv * (F_ext - D_adm_ * xdot_adm_);

  // Integrate
  xdot_adm_ += xddot_adm * dt;
  x_adm_ += xdot_adm_ * dt;

  // Clamp
  double magnitude = x_adm_.norm();
  if (magnitude > max_displacement_)
  {
    x_adm_ = x_adm_ * (max_displacement_ / magnitude);
    xdot_adm_.setZero();
  }
}

bool HybridInsertionController::checkSuccess()
{
  // Success criteria:
  // 1. Z depth reached
  double depth = getCurrentDepth();
  bool depth_reached = (depth >= z_target_);

  // 2. Force dropped (indicates full insertion)
  double force_current = std::abs(ft_sensor_.getForce().z());
  bool force_dropped = (force_initial_ - force_current) > force_drop_threshold_;

  return depth_reached && force_dropped;
}

// Implement additional pure virtuals from Controller
void HybridInsertionController::setQInit(const tum_ics_ur_robot_lli::JointState& qinit)
{
  q_init_ = qinit;
  ROS_INFO_STREAM("HybridInsertionController::setQInit() - stored " << q_init_.q.size() << " joints");
}

void HybridInsertionController::setQHome(const tum_ics_ur_robot_lli::JointState& qhome)
{
  q_home_ = qhome;

  // Update kinematic model and set center position to home EE position
  if (kinematic_model_)
  {
    Eigen::Affine3d home_pose = kinematic_model_->Tef_0(q_home_.q);
    center_ = home_pose.translation();
    z_start_ = center_.z();
    p_desired_ = center_;
    R_desired_ = home_pose.rotation();

    ROS_INFO_STREAM("HybridInsertionController::setQHome() - stored " << q_home_.q.size() << " joints");
    ROS_INFO_STREAM("Home EE position: " << center_.transpose());
  }
  else
  {
    ROS_WARN_STREAM("HybridInsertionController::setQHome() - KinematicModel not initialized yet");
  }
}

void HybridInsertionController::setQPark(const tum_ics_ur_robot_lli::JointState& qpark)
{
  q_park_ = qpark;
  ROS_INFO_STREAM("HybridInsertionController::setQPark() - stored " << q_park_.q.size() << " joints");
}

} // namespace tum_ics_ur10_controller_tutorial
