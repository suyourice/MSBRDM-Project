#include <tum_ics_ur10_controller_tutorial/operational_space_controller.h>

using namespace tum_ics_ur10_controller_tutorial;

OperationalSpaceController::OperationalSpaceController(double weight,
                                                       const QString& name)
  : ControlEffort(weight, name),
    phase_(Phase::PARK),
    radius_(0.2),
    omega_(0.4)
{}

bool OperationalSpaceController::init()
{
  Kp_x_.setIdentity();
  Kp_x_ *= 5.0;

  Kd_x_.setIdentity();
  Kd_x_ *= 2.0;

  Kp_q_.setIdentity(robot_->getDOF());
  Kp_q_ *= 2.0;

  q_home_ = robot_->qHome();
  q_safe_ = robot_->qPark();

  return true;
}

bool OperationalSpaceController::start()
{
  phase_start_ = robot_->getTime();
  phase_ = Phase::PARK;
  return true;
}

Eigen::VectorXd OperationalSpaceController::update(const RobotTime& time,
                                                   const JointState& state)
{
  const auto& q = state.q;
  const auto& qdot = state.qp;

  Eigen::VectorXd tau = Eigen::VectorXd::Zero(q.size());

  // --- phase switching ---
  double t = (time - phase_start_).toSec();

  if (phase_ == Phase::PARK && t > 2.0)
  {
    phase_ = Phase::SAFE;
    phase_start_ = time;
  }
  else if (phase_ == Phase::SAFE && t > 2.0)
  {
    phase_ = Phase::LINEAR;
    phase_start_ = time;
  }

  // --- joint space phases ---
  if (phase_ == Phase::PARK || phase_ == Phase::SAFE)
  {
    Eigen::VectorXd qd = (phase_ == Phase::PARK) ? q_home_ : q_safe_;
    tau = -Kp_q_ * (q - qd) - 0.1 * qdot;
    return tau;
  }

  // --- operational space ---
  Eigen::Affine3d x = robot_->getEndEffectorPose();
  Eigen::MatrixXd J = robot_->getJacobian().topRows(3);

  Eigen::Vector3d xdot_d = desiredCartesianVelocity(time);
  Eigen::Vector3d xdot = J * qdot;

  Eigen::Vector3d F = Kp_x_ * (xdot_d - xdot);
  tau = J.transpose() * F;

  return tau;
}

bool OperationalSpaceController::stop()
{
  return true;
}

Eigen::Vector3d
OperationalSpaceController::desiredCartesianVelocity(const RobotTime& time)
{
  double t = (time - phase_start_).toSec();
  Eigen::Vector3d v;
  v << -radius_ * omega_ * sin(omega_ * t),
        radius_ * omega_ * cos(omega_ * t),
        0.0;
  return v;
}
