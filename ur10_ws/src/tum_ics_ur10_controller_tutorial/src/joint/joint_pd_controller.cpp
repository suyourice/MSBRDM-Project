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
    max_torque_(50.0),
    transition_time_(3.0),  // Default 3 seconds for smooth transition
    is_interpolating_(false),
    first_update_(true),
    t_start_(0.0)
{
}

void JointPDController::onEnterState(const tum_ics_ur_robot_lli::RobotTime& time,
                                     const tum_ics_ur_robot_lli::JointState& state)
{
  if (state.q.size() == 0)
    return;

  q_start_ = state.q;
  q_desired_ = state.q;
  q_target_ = state.q;
  q_error_.setZero();
  is_interpolating_ = false;
  first_update_ = false;
  t_start_ = time.tD();
}

void JointPDController::onExitState()
{
  q_error_.setZero();
  is_interpolating_ = false;
}

void JointPDController::setDesiredPosition(const Eigen::VectorXd& q_desired)
{
  q_target_ = q_desired;
  // If we have a current position, start interpolating
  if (q_start_.size() > 0 && !first_update_)
  {
    is_interpolating_ = true;
    q_start_ = q_desired_;  // Start from current interpolated position
    t_start_ = -1.0;        // Will be set on next update
    ROS_INFO_STREAM("JointPDController: Starting interpolation to new target");
  }
  else
  {
    // No current position yet, just set directly
    q_desired_ = q_desired;
  }
}

bool JointPDController::isAtTarget(double tolerance) const
{
  if (q_error_.size() == 0)
    return false;

  // Interpolation complete = at target
  return !is_interpolating_;
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

  // Read parameters from ROS parameter server (private namespace)
  ros::NodeHandle pnh("~");
  std::string ns = "joint_pd_controller";

  // Read Kp gains
  std::vector<double> kp_vec;
  if (pnh.getParam(ns + "/Kp", kp_vec))
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
  if (pnh.getParam(ns + "/Kd", kd_vec))
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
  pnh.param(ns + "/max_torque", max_torque_, 50.0);

  // Read transition time for smooth interpolation
  pnh.param(ns + "/transition_time", transition_time_, 3.0);

  // Initialize target position
  q_target_ = Eigen::VectorXd::Zero(dof);

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

  // Reset interpolation state - will capture current position on first update
  first_update_ = true;
  is_interpolating_ = true;
  t_start_ = -1.0;  // Will be set on first update

  ROS_INFO_STREAM("JointPDController: Will interpolate to target over " << transition_time_ << "s");

  return true;
}

Tum::VectorDOFd JointPDController::update(
  const tum_ics_ur_robot_lli::RobotTime& time,
  const tum_ics_ur_robot_lli::JointState& state)
{
  const Eigen::VectorXd& q = state.q;
  const Eigen::VectorXd& qp = state.qp;

  // Get current time in seconds
  double t_now = time.tD();

  // On first update, capture current position as start
  if (first_update_)
  {
    q_start_ = q;
    q_desired_ = q;  // Start from current position
    t_start_ = t_now;
    first_update_ = false;
    ROS_INFO_STREAM("JointPDController: Captured start position [rad]: " << q_start_.transpose());
    ROS_INFO_STREAM("JointPDController: Target position [rad]: " << q_target_.transpose());
  }

  // Interpolation logic
  if (is_interpolating_)
  {
    // Handle case where t_start_ was reset by setDesiredPosition
    if (t_start_ < 0)
    {
      t_start_ = t_now;
    }

    double t_elapsed = t_now - t_start_;
    double alpha = t_elapsed / transition_time_;

    if (alpha >= 1.0)
    {
      // Interpolation complete
      alpha = 1.0;
      is_interpolating_ = false;
      q_desired_ = q_target_;
      ROS_INFO_STREAM("JointPDController: Interpolation complete, at target position");
    }
    else
    {
      // Smooth interpolation using cosine (S-curve)
      // alpha_smooth goes 0→1 smoothly with zero velocity at endpoints
      double alpha_smooth = 0.5 * (1.0 - std::cos(M_PI * alpha));
      q_desired_ = (1.0 - alpha_smooth) * q_start_ + alpha_smooth * q_target_;
    }
  }

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
  // Set target position to home position (interpolation will handle smooth transition)
  q_target_ = q_home_.q;
  // Also set q_desired_ for initial state (before first update)
  if (first_update_)
  {
    q_desired_ = q_home_.q;
  }
  ROS_INFO_STREAM("JointPDController::setQHome() - stored " << q_home_.q.size() << " joints");
  ROS_INFO_STREAM("Home position [rad]: " << q_home_.q.transpose());
}

void JointPDController::setQPark(const tum_ics_ur_robot_lli::JointState& qpark)
{
  q_park_ = qpark;
  ROS_INFO_STREAM("JointPDController::setQPark() - stored " << q_park_.q.size() << " joints");
}

} // namespace tum_ics_ur10_controller_tutorial
