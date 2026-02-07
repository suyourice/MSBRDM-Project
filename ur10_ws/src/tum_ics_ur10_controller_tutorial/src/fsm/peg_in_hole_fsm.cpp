#include <tum_ics_ur10_controller_tutorial/fsm/peg_in_hole_fsm.h>

namespace tum_ics_ur10_controller_tutorial
{

PegInHoleFSM::PegInHoleFSM(double weight, const QString& name)
  : ControllerBase(weight, name),
    joint_ctrl_(1.0, "FSM_JointPD"),
    cartesian_ctrl_(1.0, "FSM_CartesianPDG"),
    insertion_ctrl_(1.0, "FSM_HybridInsertion"),
    state_(State::INIT),
    retry_count_(0),
    max_retries_(3),
    use_aruco_(false)
{
}

std::string PegInHoleFSM::getStateName() const
{
  switch (state_)
  {
    case State::INIT: return "INIT";
    case State::HOME: return "HOME";
    case State::OPEN_GRIPPER: return "OPEN_GRIPPER";
    case State::MOVE_TO_PEG: return "MOVE_TO_PEG";
    case State::DESCEND_TO_PEG: return "DESCEND_TO_PEG";
    case State::CLOSE_GRIPPER: return "CLOSE_GRIPPER";
    case State::LIFT_PEG: return "LIFT_PEG";
    case State::MOVE_ABOVE_HOLE: return "MOVE_ABOVE_HOLE";
    case State::ALIGN_ORIENTATION: return "ALIGN_ORIENTATION";
    case State::CIRCULAR_INSERTION: return "CIRCULAR_INSERTION";
    case State::SUCCESS: return "SUCCESS";
    case State::RETRY: return "RETRY";
    case State::DONE: return "DONE";
    default: return "UNKNOWN";
  }
}

bool PegInHoleFSM::init()
{
  ROS_INFO_STREAM("PegInHoleFSM::init()");

  // Set robot for sub-controllers
  joint_ctrl_.setRobot(robot_);
  cartesian_ctrl_.setRobot(robot_);
  insertion_ctrl_.setRobot(robot_);

  // Initialize sub-controllers
  if (!joint_ctrl_.init() || !cartesian_ctrl_.init() || !insertion_ctrl_.init())
  {
    ROS_ERROR("Failed to initialize sub-controllers");
    return false;
  }

  // Read waypoints from config
  std::string ns = "~peg_in_hole_fsm";

  // Home position
  std::vector<double> q_home_vec;
  if (nh_.getParam(ns + "/q_home", q_home_vec))
  {
    q_home_ = Eigen::VectorXd(q_home_vec.size());
    for (size_t i = 0; i < q_home_vec.size(); ++i)
      q_home_(i) = q_home_vec[i] * M_PI / 180.0;  // Convert deg to rad
  }
  else
  {
    q_home_ = robot_->qHome();
  }

  // Peg position
  std::vector<double> p_peg_vec;
  if (nh_.getParam(ns + "/p_peg", p_peg_vec) && p_peg_vec.size() == 3)
  {
    p_peg_ = Eigen::Vector3d(p_peg_vec.data());
  }
  else
  {
    ROS_WARN("Peg position not specified, using default");
    p_peg_ << 0.5, 0.2, 0.15;
  }

  // Hole position
  std::vector<double> p_hole_vec;
  if (nh_.getParam(ns + "/p_hole", p_hole_vec) && p_hole_vec.size() == 3)
  {
    p_hole_ = Eigen::Vector3d(p_hole_vec.data());
  }
  else
  {
    ROS_WARN("Hole position not specified, using default");
    p_hole_ << 0.5, -0.2, 0.15;
  }

  // Peg orientation (RPY in degrees)
  std::vector<double> rpy_vec;
  if (nh_.getParam(ns + "/peg_orientation", rpy_vec) && rpy_vec.size() == 3)
  {
    double roll = rpy_vec[0] * M_PI / 180.0;
    double pitch = rpy_vec[1] * M_PI / 180.0;
    double yaw = rpy_vec[2] * M_PI / 180.0;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    R_peg_ = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
  }
  else
  {
    R_peg_.setIdentity();
  }

  // State timeouts
  state_timeouts_[State::HOME] = 10.0;
  state_timeouts_[State::OPEN_GRIPPER] = 2.0;
  state_timeouts_[State::MOVE_TO_PEG] = 15.0;
  state_timeouts_[State::DESCEND_TO_PEG] = 10.0;
  state_timeouts_[State::CLOSE_GRIPPER] = 2.0;
  state_timeouts_[State::LIFT_PEG] = 5.0;
  state_timeouts_[State::MOVE_ABOVE_HOLE] = 15.0;
  state_timeouts_[State::ALIGN_ORIENTATION] = 5.0;
  state_timeouts_[State::CIRCULAR_INSERTION] = 30.0;

  // Other parameters
  nh_.param(ns + "/max_retries", max_retries_, 3);
  nh_.param(ns + "/use_aruco", use_aruco_, false);

  // Initialize gripper
  if (!gripper_.init(nh_))
  {
    ROS_ERROR("Failed to initialize gripper");
    return false;
  }

  ROS_INFO_STREAM("PegInHoleFSM initialized");
  ROS_INFO_STREAM("  Peg position: " << p_peg_.transpose());
  ROS_INFO_STREAM("  Hole position: " << p_hole_.transpose());
  ROS_INFO_STREAM("  Max retries: " << max_retries_);

  return true;
}

bool PegInHoleFSM::start()
{
  ROS_INFO_STREAM("PegInHoleFSM::start()");

  retry_count_ = 0;
  transitionTo(State::HOME);

  return true;
}

bool PegInHoleFSM::stop()
{
  ROS_INFO_STREAM("PegInHoleFSM::stop()");
  return true;
}

void PegInHoleFSM::transitionTo(State new_state)
{
  std::string old_state_name = getStateName();
  state_ = new_state;
  ROS_INFO_STREAM("FSM: " << old_state_name << " -> " << getStateName());

  state_start_time_ = robot_->getTime();

  // State entry actions
  switch (state_)
  {
    case State::HOME:
      joint_ctrl_.start();
      joint_ctrl_.setDesiredPosition(q_home_);
      break;

    case State::OPEN_GRIPPER:
      gripper_.open();
      break;

    case State::MOVE_TO_PEG:
    {
      cartesian_ctrl_.start();
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_peg_;
      target.translation().z() += 0.1;  // 10cm above
      target.linear() = R_peg_;
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::DESCEND_TO_PEG:
    {
      cartesian_ctrl_.start();
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_peg_;
      target.linear() = R_peg_;
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::CLOSE_GRIPPER:
      gripper_.grasp(20.0);  // 20N grasping force
      break;

    case State::LIFT_PEG:
    {
      cartesian_ctrl_.start();
      Eigen::Affine3d target = cartesian_ctrl_.getDesiredPose();
      target.translation().z() += 0.05;  // Lift 5cm
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::MOVE_ABOVE_HOLE:
    {
      cartesian_ctrl_.start();
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_hole_;
      target.translation().z() += 0.1;  // 10cm above hole
      target.linear() = R_peg_;
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::ALIGN_ORIENTATION:
    {
      cartesian_ctrl_.start();
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_hole_;
      target.translation().z() += 0.05;  // 5cm above hole
      target.linear() = R_peg_;
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::CIRCULAR_INSERTION:
    {
      insertion_ctrl_.start();
      Eigen::Vector3d start_pos = p_hole_;
      start_pos.z() += 0.02;  // Start 2cm above
      insertion_ctrl_.setStartPosition(start_pos);
      insertion_ctrl_.setTargetDepth(0.025);  // 25mm insertion
      insertion_ctrl_.reset();
      break;
    }

    default:
      break;
  }
}

bool PegInHoleFSM::checkStateComplete()
{
  double elapsed = (robot_->getTime() - state_start_time_).toSec();

  // Check timeout
  if (state_timeouts_.count(state_) > 0)
  {
    if (elapsed > state_timeouts_[state_])
    {
      ROS_WARN_STREAM("State " << getStateName() << " timed out after " << elapsed << "s");
      return true;
    }
  }

  // State-specific completion checks
  switch (state_)
  {
    case State::HOME:
      return joint_ctrl_.isAtTarget();

    case State::MOVE_TO_PEG:
    case State::DESCEND_TO_PEG:
    case State::LIFT_PEG:
    case State::MOVE_ABOVE_HOLE:
    case State::ALIGN_ORIENTATION:
      return cartesian_ctrl_.isAtTarget();

    case State::OPEN_GRIPPER:
    case State::CLOSE_GRIPPER:
      return elapsed > 2.0;  // Wait for gripper action

    case State::CIRCULAR_INSERTION:
      return insertion_ctrl_.isInsertionComplete();

    default:
      return false;
  }
}

Eigen::VectorXd PegInHoleFSM::update(const RobotTime& time, const JointState& state)
{
  // State machine logic - check for transitions
  if (checkStateComplete())
  {
    switch (state_)
    {
      case State::HOME:
        transitionTo(State::OPEN_GRIPPER);
        break;

      case State::OPEN_GRIPPER:
        transitionTo(State::MOVE_TO_PEG);
        break;

      case State::MOVE_TO_PEG:
        transitionTo(State::DESCEND_TO_PEG);
        break;

      case State::DESCEND_TO_PEG:
        transitionTo(State::CLOSE_GRIPPER);
        break;

      case State::CLOSE_GRIPPER:
        transitionTo(State::LIFT_PEG);
        break;

      case State::LIFT_PEG:
        transitionTo(State::MOVE_ABOVE_HOLE);
        break;

      case State::MOVE_ABOVE_HOLE:
        transitionTo(State::ALIGN_ORIENTATION);
        break;

      case State::ALIGN_ORIENTATION:
        transitionTo(State::CIRCULAR_INSERTION);
        break;

      case State::CIRCULAR_INSERTION:
        if (insertion_ctrl_.isInsertionComplete())
        {
          ROS_INFO("Insertion successful!");
          transitionTo(State::SUCCESS);
        }
        else
        {
          // Timed out - retry
          if (retry_count_ < max_retries_)
          {
            retry_count_++;
            ROS_WARN_STREAM("Insertion failed, retrying (" << retry_count_ << "/" << max_retries_ << ")");
            transitionTo(State::LIFT_PEG);  // Go back and try again
          }
          else
          {
            ROS_ERROR("Max retries exceeded, aborting");
            transitionTo(State::DONE);
          }
        }
        break;

      case State::SUCCESS:
        transitionTo(State::DONE);
        break;

      default:
        break;
    }
  }

  // Generate control torques based on current state
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(state.q.size());

  switch (state_)
  {
    case State::HOME:
      tau = joint_ctrl_.update(time, state);
      break;

    case State::MOVE_TO_PEG:
    case State::DESCEND_TO_PEG:
    case State::LIFT_PEG:
    case State::MOVE_ABOVE_HOLE:
    case State::ALIGN_ORIENTATION:
      tau = cartesian_ctrl_.update(time, state);
      break;

    case State::CIRCULAR_INSERTION:
      tau = insertion_ctrl_.update(time, state);
      break;

    default:
      tau.setZero();
      break;
  }

  return tau;
}

} // namespace tum_ics_ur10_controller_tutorial
