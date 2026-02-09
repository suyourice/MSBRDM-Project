#include <tum_ics_ur10_controller_tutorial/fsm/peg_in_hole_fsm.h>
#include <XmlRpcValue.h>

namespace tum_ics_ur10_controller_tutorial
{

PegInHoleFSM::PegInHoleFSM(double weight, const QString& name)
  : ControllerBase(name,
                   tum_ics_ur_robot_lli::RobotControllers::STANDARD_TYPE,
                   tum_ics_ur_robot_lli::RobotControllers::JOINT_SPACE,
                   weight),
    joint_ctrl_("FSM_JointPD"),
    cartesian_ctrl_("FSM_CartesianPDG"),
    state_(State::INIT),
    pending_transition_(State::INIT),
    has_pending_transition_(false),
    state_cycles_(0),
    target_pending_(false),
    retry_count_(0),
    max_retries_(3),
    use_aruco_(false),
    has_q_safe_param_(false),
    use_frame_transform_(false),
    frame_R_(Eigen::Matrix3d::Identity()),
    frame_t_(Eigen::Vector3d::Zero()),
    use_tool_transform_(false),
    tool_T_(Eigen::Affine3d::Identity()),
    min_dwell_joint_to_cartesian_(0.3),
    min_dwell_cartesian_(0.1),
    hold_cartesian_time_(0.3),
    last_tau_valid_(false),
    insertion_radius_(0.003),
    insertion_omega_(0.5),
    insertion_v_descent_(0.002),
    insertion_target_depth_(0.025),
    insertion_success_(false)
{
  last_tau_ = Eigen::VectorXd::Zero(6);
  insertion_start_pos_.setZero();
}

std::string PegInHoleFSM::getStateName() const
{
  switch (state_)
  {
    case State::INIT: return "INIT";
    case State::MOVE_TO_SAFE: return "MOVE_TO_SAFE";
    case State::OPEN_GRIPPER: return "OPEN_GRIPPER";
    case State::HOLD_CARTESIAN: return "HOLD_CARTESIAN";
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

  // Initialize sub-controllers (they create their own KinematicModels)
  // CRITICAL: Only initialize joint_ctrl_ and cartesian_ctrl_ (no insertion_ctrl_!)
  if (!joint_ctrl_.callInit() || !cartesian_ctrl_.callInit())
  {
    ROS_ERROR("Failed to initialize sub-controllers");
    return false;
  }

  // Read waypoints from config (private namespace)
  ros::NodeHandle pnh("~");
  std::string ns = "peg_in_hole_fsm";

  // Safe position (user-defined starting position)
  std::vector<double> q_safe_vec;
  if (pnh.getParam(ns + "/q_safe", q_safe_vec))
  {
    q_safe_ = Eigen::VectorXd(q_safe_vec.size());
    for (size_t i = 0; i < q_safe_vec.size(); ++i)
      q_safe_(i) = q_safe_vec[i] * M_PI / 180.0;  // Convert deg to rad
    has_q_safe_param_ = true;
    ROS_INFO_STREAM("Using q_safe from YAML [deg]: "
                    << Eigen::Map<Eigen::VectorXd>(q_safe_vec.data(), q_safe_vec.size()).transpose());
  }
  else
  {
    // Default safe position (all zeros)
    q_safe_ = Eigen::VectorXd::Zero(6);
    ROS_WARN("Safe position not specified, using zeros");
    has_q_safe_param_ = false;
  }

  // Peg position
  std::vector<double> p_peg_vec;
  if (pnh.getParam(ns + "/p_peg", p_peg_vec) && p_peg_vec.size() == 3)
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
  if (pnh.getParam(ns + "/p_hole", p_hole_vec) && p_hole_vec.size() == 3)
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
  if (pnh.getParam(ns + "/peg_orientation", rpy_vec) && rpy_vec.size() == 3)
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

  // Optional frame transform: map visual/URDF frame -> controller frame
  // p_controller = R * p_visual + t
  pnh.param(ns + "/use_frame_transform", use_frame_transform_, false);
  if (use_frame_transform_)
  {
    std::vector<double> frame_rpy_deg;
    std::vector<double> frame_xyz;

    if (pnh.getParam(ns + "/frame_rpy_deg", frame_rpy_deg) && frame_rpy_deg.size() == 3)
    {
      double rr = frame_rpy_deg[0] * M_PI / 180.0;
      double rp = frame_rpy_deg[1] * M_PI / 180.0;
      double ry = frame_rpy_deg[2] * M_PI / 180.0;
      Eigen::AngleAxisd r_roll(rr, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd r_pitch(rp, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd r_yaw(ry, Eigen::Vector3d::UnitZ());
      frame_R_ = (r_yaw * r_pitch * r_roll).toRotationMatrix();
    }

    if (pnh.getParam(ns + "/frame_xyz", frame_xyz) && frame_xyz.size() == 3)
    {
      frame_t_ = Eigen::Vector3d(frame_xyz[0], frame_xyz[1], frame_xyz[2]);
    }

    const Eigen::Vector3d p_peg_raw = p_peg_;
    const Eigen::Vector3d p_hole_raw = p_hole_;

    p_peg_ = frame_R_ * p_peg_raw + frame_t_;
    p_hole_ = frame_R_ * p_hole_raw + frame_t_;
    R_peg_ = frame_R_ * R_peg_;

    ROS_WARN_STREAM("Frame transform enabled: p_controller = R * p_visual + t");
    if (frame_rpy_deg.size() == 3)
    {
      ROS_INFO_STREAM("  frame_rpy_deg: [" << frame_rpy_deg[0] << ", "
                                           << frame_rpy_deg[1] << ", "
                                           << frame_rpy_deg[2] << "]");
    }
    ROS_INFO_STREAM("  frame_xyz: [" << frame_t_.transpose() << "]");
    ROS_INFO_STREAM("  p_peg (raw): " << p_peg_raw.transpose());
    ROS_INFO_STREAM("  p_peg (xfm): " << p_peg_.transpose());
    ROS_INFO_STREAM("  p_hole(raw): " << p_hole_raw.transpose());
    ROS_INFO_STREAM("  p_hole(xfm): " << p_hole_.transpose());
  }

  // Optional tool transform: ee -> tool (e.g., gripper base)
  // If provided, we will command ee pose such that the tool pose matches target.
  std::vector<double> tool_rpy_deg;
  std::vector<double> tool_xyz;
  bool has_tool_rpy = pnh.getParam(ns + "/tool_rpy_deg", tool_rpy_deg) && tool_rpy_deg.size() == 3;
  bool has_tool_xyz = pnh.getParam(ns + "/tool_xyz", tool_xyz) && tool_xyz.size() == 3;
  if (has_tool_rpy || has_tool_xyz)
  {
    use_tool_transform_ = true;
    Eigen::Matrix3d R_tool = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_tool = Eigen::Vector3d::Zero();

    if (has_tool_rpy)
    {
      double rr = tool_rpy_deg[0] * M_PI / 180.0;
      double rp = tool_rpy_deg[1] * M_PI / 180.0;
      double ry = tool_rpy_deg[2] * M_PI / 180.0;
      Eigen::AngleAxisd r_roll(rr, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd r_pitch(rp, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd r_yaw(ry, Eigen::Vector3d::UnitZ());
      R_tool = (r_yaw * r_pitch * r_roll).toRotationMatrix();
    }

    if (has_tool_xyz)
    {
      t_tool = Eigen::Vector3d(tool_xyz[0], tool_xyz[1], tool_xyz[2]);
    }

    tool_T_ = Eigen::Affine3d::Identity();
    tool_T_.linear() = R_tool;
    tool_T_.translation() = t_tool;

    ROS_WARN_STREAM("Tool transform enabled (ee -> tool)");
    if (has_tool_rpy)
    {
      ROS_INFO_STREAM("  tool_rpy_deg: [" << tool_rpy_deg[0] << ", "
                                          << tool_rpy_deg[1] << ", "
                                          << tool_rpy_deg[2] << "]");
    }
    if (has_tool_xyz)
    {
      ROS_INFO_STREAM("  tool_xyz: [" << t_tool.transpose() << "]");
    }
  }

  // State timeouts (seconds)
  state_timeouts_[State::MOVE_TO_SAFE] = 10.0;
  state_timeouts_[State::OPEN_GRIPPER] = 2.0;
  state_timeouts_[State::HOLD_CARTESIAN] = hold_cartesian_time_;
  state_timeouts_[State::MOVE_TO_PEG] = 30.0;
  state_timeouts_[State::DESCEND_TO_PEG] = 30.0;
  state_timeouts_[State::CLOSE_GRIPPER] = 2.0;
  state_timeouts_[State::LIFT_PEG] = 10.0;
  state_timeouts_[State::MOVE_ABOVE_HOLE] = 30.0;
  state_timeouts_[State::ALIGN_ORIENTATION] = 10.0;
  state_timeouts_[State::CIRCULAR_INSERTION] = 30.0;

  // Optional timeout overrides from YAML:
  // peg_in_hole_fsm:
  //   timeouts:
  //     MOVE_TO_PEG: 30.0
  //     DESCEND_TO_PEG: 30.0
  XmlRpc::XmlRpcValue timeouts;
  if (pnh.getParam(ns + "/timeouts", timeouts) &&
      timeouts.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    auto read_timeout = [&](State st, const std::string& key) {
      if (timeouts.hasMember(key))
      {
        const XmlRpc::XmlRpcValue& v = timeouts[key];
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
          state_timeouts_[st] = static_cast<int>(v);
        else if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
          state_timeouts_[st] = static_cast<double>(v);
      }
    };
    read_timeout(State::MOVE_TO_SAFE, "MOVE_TO_SAFE");
    read_timeout(State::OPEN_GRIPPER, "OPEN_GRIPPER");
    read_timeout(State::HOLD_CARTESIAN, "HOLD_CARTESIAN");
    read_timeout(State::MOVE_TO_PEG, "MOVE_TO_PEG");
    read_timeout(State::DESCEND_TO_PEG, "DESCEND_TO_PEG");
    read_timeout(State::CLOSE_GRIPPER, "CLOSE_GRIPPER");
    read_timeout(State::LIFT_PEG, "LIFT_PEG");
    read_timeout(State::MOVE_ABOVE_HOLE, "MOVE_ABOVE_HOLE");
    read_timeout(State::ALIGN_ORIENTATION, "ALIGN_ORIENTATION");
    read_timeout(State::CIRCULAR_INSERTION, "CIRCULAR_INSERTION");
  }

  // Other parameters
  pnh.param(ns + "/max_retries", max_retries_, 3);
  pnh.param(ns + "/use_aruco", use_aruco_, false);
  pnh.param(ns + "/min_dwell_joint_to_cartesian", min_dwell_joint_to_cartesian_, 0.3);
  pnh.param(ns + "/min_dwell_cartesian", min_dwell_cartesian_, 0.1);
  pnh.param(ns + "/hold_cartesian_time", hold_cartesian_time_, 0.3);
  state_timeouts_[State::HOLD_CARTESIAN] = hold_cartesian_time_;

  // Circular insertion parameters (FSM-managed trajectory)
  pnh.param(ns + "/insertion_radius", insertion_radius_, 0.003);  // 3mm
  pnh.param(ns + "/insertion_omega", insertion_omega_, 0.5);      // 0.5 rad/s
  pnh.param(ns + "/insertion_v_descent", insertion_v_descent_, 0.002);  // 2mm/s
  pnh.param(ns + "/insertion_target_depth", insertion_target_depth_, 0.025);  // 25mm

  // Initialize gripper
  if (!gripper_.init(pnh))
  {
    ROS_ERROR("Failed to initialize gripper");
    return false;
  }

  // If q_safe was loaded from YAML, propagate to sub-controllers now
  if (has_q_safe_param_)
  {
    // Create JointState with YAML q_safe
    tum_ics_ur_robot_lli::JointState q_safe_state;
    q_safe_state.q = q_safe_;
    q_safe_state.qp = Eigen::VectorXd::Zero(q_safe_.size());
    q_safe_state.qpp = Eigen::VectorXd::Zero(q_safe_.size());
    q_safe_state.tau = Eigen::VectorXd::Zero(q_safe_.size());

    q_home_stored_ = q_safe_state;  // Store as home for sub-controllers
    joint_ctrl_.setQHome(q_safe_state);
    cartesian_ctrl_.setQHome(q_safe_state);

    ROS_INFO_STREAM("Propagated YAML q_safe to sub-controllers [rad]: " << q_safe_.transpose());
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
  state_ = State::INIT;
  has_pending_transition_ = false;
  target_pending_ = false;
  state_cycles_ = 0;

  return true;
}

bool PegInHoleFSM::stop()
{
  ROS_INFO_STREAM("PegInHoleFSM::stop()");
  return true;
}

void PegInHoleFSM::switchState(State new_state,
                               const tum_ics_ur_robot_lli::RobotTime& time,
                               const tum_ics_ur_robot_lli::JointState& state)
{
  // Exit hook for the current controller
  // CRITICAL: Only two controllers used (joint_ctrl_, cartesian_ctrl_)
  switch (state_)
  {
    case State::MOVE_TO_SAFE:
    case State::OPEN_GRIPPER:
      joint_ctrl_.onExitState();
      break;

    case State::MOVE_TO_PEG:
    case State::DESCEND_TO_PEG:
    case State::LIFT_PEG:
    case State::MOVE_ABOVE_HOLE:
    case State::ALIGN_ORIENTATION:
    case State::CLOSE_GRIPPER:
      cartesian_ctrl_.onExitState();
      break;

    case State::CIRCULAR_INSERTION:
      // Return to TRACKING mode after insertion
      cartesian_ctrl_.setMode(CartesianPDGController::Mode::TRACKING);
      cartesian_ctrl_.onExitState();
      ROS_INFO_STREAM("FSM: Exiting CIRCULAR_INSERTION, mode set to TRACKING");
      break;

    default:
      break;
  }

  const std::string old_state_name = getStateName();
  state_ = new_state;
  ROS_INFO_STREAM("FSM: " << old_state_name << " -> " << getStateName());

  state_start_time_ = time;
  state_cycles_ = 0;
  target_pending_ = true;

  // State entry actions
  // CRITICAL: Don't call callStart() - controllers stay alive!
  // Only call onEnterState() to sync state
  switch (state_)
  {
    case State::MOVE_TO_SAFE:
      // First time using joint controller - call start once
      if (old_state_name == "INIT")
        joint_ctrl_.callStart();
      joint_ctrl_.onEnterState(time, state);
      break;

    case State::OPEN_GRIPPER:
      // Joint controller continues from MOVE_TO_SAFE
      joint_ctrl_.onEnterState(time, state);
      break;

    case State::HOLD_CARTESIAN:
      // First time using Cartesian controller - call start once
      cartesian_ctrl_.callStart();
      cartesian_ctrl_.onEnterState(time, state);
      break;

    case State::MOVE_TO_PEG:
    {
      // Cartesian controller continues - only sync state
      cartesian_ctrl_.onEnterState(time, state);
      break;
    }

    case State::DESCEND_TO_PEG:
    {
      // Cartesian controller continues
      cartesian_ctrl_.onEnterState(time, state);
      break;
    }

    case State::CLOSE_GRIPPER:
      // Cartesian controller continues
      cartesian_ctrl_.onEnterState(time, state);
      break;

    case State::LIFT_PEG:
    {
      // Cartesian controller continues
      cartesian_ctrl_.onEnterState(time, state);
      break;
    }

    case State::MOVE_ABOVE_HOLE:
    {
      // Cartesian controller continues
      cartesian_ctrl_.onEnterState(time, state);
      break;
    }

    case State::ALIGN_ORIENTATION:
    {
      // Cartesian controller continues
      cartesian_ctrl_.onEnterState(time, state);
      break;
    }

    case State::CIRCULAR_INSERTION:
    {
      // CRITICAL: Continue using cartesian_ctrl_ (no controller switch!)
      // Just sync state for smooth transition
      cartesian_ctrl_.onEnterState(time, state);

      // Initialize insertion trajectory timing
      insertion_start_time_ = time.tRc().toSec();
      insertion_success_ = false;
      break;
    }

    default:
      break;
  }
}

void PegInHoleFSM::applyStateTargets(const tum_ics_ur_robot_lli::JointState& state)
{
  switch (state_)
  {
    case State::MOVE_TO_SAFE:
      joint_ctrl_.setDesiredPosition(q_safe_);
      break;

    case State::OPEN_GRIPPER:
      gripper_.open();
      break;

    case State::HOLD_CARTESIAN:
      // Do NOT change target - onEnterState() already synced to current EE
      // Just ensure zero desired velocity
      cartesian_ctrl_.setDesiredVelocity(Eigen::Matrix<double, 6, 1>::Zero());
      break;

    case State::MOVE_TO_PEG:
    {
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_peg_;
      target.translation().z() += 0.1;
      target.linear() = R_peg_;
      if (use_tool_transform_)
      {
        target = target * tool_T_.inverse();
      }
      ROS_INFO_STREAM("MOVE_TO_PEG target position: " << target.translation().transpose());
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::DESCEND_TO_PEG:
    {
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_peg_;
      target.linear() = R_peg_;
      if (use_tool_transform_)
      {
        target = target * tool_T_.inverse();
      }
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::CLOSE_GRIPPER:
      // Hold current position while gripper closes
      // (onEnterState already synced to current EE)
      cartesian_ctrl_.setDesiredVelocity(Eigen::Matrix<double, 6, 1>::Zero());
      gripper_.grasp(20.0);
      break;

    case State::LIFT_PEG:
    {
      Eigen::Affine3d target = cartesian_ctrl_.getDesiredPose();
      target.translation().z() += 0.05;
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::MOVE_ABOVE_HOLE:
    {
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_hole_;
      target.translation().z() += 0.1;
      target.linear() = R_peg_;
      if (use_tool_transform_)
      {
        target = target * tool_T_.inverse();
      }
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::ALIGN_ORIENTATION:
    {
      Eigen::Affine3d target = Eigen::Affine3d::Identity();
      target.translation() = p_hole_;
      target.translation().z() += 0.05;
      target.linear() = R_peg_;
      if (use_tool_transform_)
      {
        target = target * tool_T_.inverse();
      }
      cartesian_ctrl_.setDesiredPose(target);
      break;
    }

    case State::CIRCULAR_INSERTION:
    {
      // Switch to INSERTION_ADMITTANCE mode (trajectory and force compliance handled by controller)
      cartesian_ctrl_.setMode(CartesianPDGController::Mode::INSERTION_ADMITTANCE);

      // Set starting position for circular motion
      Eigen::Vector3d insertion_center = p_hole_;
      insertion_center.z() += 0.02;  // Start 2cm above hole
      cartesian_ctrl_.setInsertionCenter(insertion_center);

      ROS_INFO_STREAM("FSM: CIRCULAR_INSERTION mode activated");
      ROS_INFO_STREAM("  Insertion center: " << insertion_center.transpose());
      break;
    }

    default:
      break;
  }
}

bool PegInHoleFSM::checkStateComplete(const tum_ics_ur_robot_lli::RobotTime& time)
{
  double elapsed = (time.tRc() - state_start_time_.tRc()).toSec();

  // CRITICAL: Prevent immediate completion after state entry
  // Need at least 2 cycles:
  //   Cycle 0: transition tick (hold last_tau)
  //   Cycle 1: apply targets
  //   Cycle 2+: can check completion
  if (state_cycles_ < 2)
    return false;

  // Minimum dwell time enforcement (physical continuity)
  const double MIN_DWELL = 0.05;  // 50ms absolute minimum

  if (state_ == State::MOVE_TO_SAFE || state_ == State::OPEN_GRIPPER)
  {
    // Joint → Cartesian transition: need more settling time
    if (elapsed < std::max(min_dwell_joint_to_cartesian_, MIN_DWELL))
      return false;
  }
  else if (state_ == State::HOLD_CARTESIAN)
  {
    // Explicit hold state: use configured time
    if (elapsed < std::max(hold_cartesian_time_, MIN_DWELL))
      return false;
  }
  else
  {
    // All other states: minimum dwell
    if (elapsed < std::max(min_dwell_cartesian_, MIN_DWELL))
      return false;
  }

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
    case State::MOVE_TO_SAFE:
      return joint_ctrl_.isAtTarget();

    case State::HOLD_CARTESIAN:
      return elapsed >= hold_cartesian_time_;

    case State::MOVE_TO_PEG:
    case State::DESCEND_TO_PEG:
    case State::LIFT_PEG:
    case State::MOVE_ABOVE_HOLE:
      // Position-only completion (orientation is not controlled in PDG config)
      return cartesian_ctrl_.isAtTarget(0.01, 1000.0);

    case State::ALIGN_ORIENTATION:
      return cartesian_ctrl_.isAtTarget();

    case State::OPEN_GRIPPER:
    case State::CLOSE_GRIPPER:
      return elapsed > 2.0;  // Wait for gripper action

    case State::CIRCULAR_INSERTION:
    {
      // Check insertion completion (z_depth and force_drop criteria)
      insertion_success_ = cartesian_ctrl_.isInsertionComplete();

      if (insertion_success_)
      {
        ROS_INFO_STREAM("CIRCULAR_INSERTION: Insertion complete");
        ROS_INFO_STREAM("  Depth: " << cartesian_ctrl_.getCurrentDepth() << "m");
        return true;
      }

      return false;
    }

    default:
      return false;
  }
}

Tum::VectorDOFd PegInHoleFSM::update(const tum_ics_ur_robot_lli::RobotTime& time,
                                     const tum_ics_ur_robot_lli::JointState& state)
{
  // Update time tracking
  last_time_ = time;

  // ============================================================================
  // CRITICAL: ONE state transition per cycle
  // During transition tick: switchState() + return last_tau (NO new computation)
  // ============================================================================

  // Apply any pending transition at the START of the cycle
  if (has_pending_transition_)
  {
    switchState(pending_transition_, time, state);
    has_pending_transition_ = false;

    // HOLD last torque during transition tick (physical continuity!)
    if (last_tau_valid_ && last_tau_.size() == state.q.size())
    {
      ROS_INFO_STREAM("FSM: Transition tick - holding last torque");
      return last_tau_;
    }
    else
    {
      return Eigen::VectorXd::Zero(state.q.size());
    }
  }

  // Handle INIT state
  if (state_ == State::INIT)
  {
    switchState(State::MOVE_TO_SAFE, time, state);
    return Eigen::VectorXd::Zero(state.q.size());
  }

  // Apply targets ONE cycle after state entry (deferred application)
  if (target_pending_ && state_cycles_ >= 1)
  {
    applyStateTargets(state);
    target_pending_ = false;
    ROS_INFO_STREAM("FSM: Targets applied for state " << getStateName());
  }

  // Generate control torques based on current state
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(state.q.size());

  switch (state_)
  {
    case State::MOVE_TO_SAFE:
      tau = joint_ctrl_.callUpdate(time, state);
      break;

    case State::OPEN_GRIPPER:
      // Hold pose with existing JointPD target (q_safe_) while gripper opens
      tau = joint_ctrl_.callUpdate(time, state);
      break;

    case State::HOLD_CARTESIAN:
    case State::MOVE_TO_PEG:
    case State::DESCEND_TO_PEG:
    case State::LIFT_PEG:
    case State::MOVE_ABOVE_HOLE:
    case State::ALIGN_ORIENTATION:
      tau = cartesian_ctrl_.callUpdate(time, state);
      break;

    case State::CLOSE_GRIPPER:
      tau = cartesian_ctrl_.callUpdate(time, state);
      break;

    case State::CIRCULAR_INSERTION:
    {
      // Controller generates trajectory and applies force compliance internally
      tau = cartesian_ctrl_.callUpdate(time, state);
      break;
    }

    default:
      tau.setZero();
      break;
  }

  // State machine logic - check for transitions AFTER update (uses fresh errors)
  if (!has_pending_transition_ && checkStateComplete(time))
  {
    switch (state_)
    {
      case State::MOVE_TO_SAFE:
        pending_transition_ = State::OPEN_GRIPPER;
        has_pending_transition_ = true;
        break;

      case State::OPEN_GRIPPER:
        pending_transition_ = State::HOLD_CARTESIAN;
        has_pending_transition_ = true;
        break;

      case State::HOLD_CARTESIAN:
        pending_transition_ = State::MOVE_TO_PEG;
        has_pending_transition_ = true;
        break;

      case State::MOVE_TO_PEG:
        pending_transition_ = State::DESCEND_TO_PEG;
        has_pending_transition_ = true;
        break;

      case State::DESCEND_TO_PEG:
        pending_transition_ = State::CLOSE_GRIPPER;
        has_pending_transition_ = true;
        break;

      case State::CLOSE_GRIPPER:
        pending_transition_ = State::LIFT_PEG;
        has_pending_transition_ = true;
        break;

      case State::LIFT_PEG:
        pending_transition_ = State::MOVE_ABOVE_HOLE;
        has_pending_transition_ = true;
        break;

      case State::MOVE_ABOVE_HOLE:
        pending_transition_ = State::ALIGN_ORIENTATION;
        has_pending_transition_ = true;
        break;

      case State::ALIGN_ORIENTATION:
        pending_transition_ = State::CIRCULAR_INSERTION;
        has_pending_transition_ = true;
        break;

      case State::CIRCULAR_INSERTION:
        if (insertion_success_)
        {
          ROS_INFO("Insertion successful!");
          pending_transition_ = State::SUCCESS;
          has_pending_transition_ = true;
        }
        else
        {
          // Timed out - retry
          if (retry_count_ < max_retries_)
          {
            retry_count_++;
            ROS_WARN_STREAM("Insertion failed, retrying (" << retry_count_ << "/" << max_retries_ << ")");
            pending_transition_ = State::LIFT_PEG;
            has_pending_transition_ = true;
          }
          else
          {
            ROS_ERROR("Max retries exceeded, aborting");
            pending_transition_ = State::DONE;
            has_pending_transition_ = true;
          }
        }
        break;

      case State::SUCCESS:
        pending_transition_ = State::DONE;
        has_pending_transition_ = true;
        break;

      default:
        break;
    }
  }

  // Store torque for next cycle's continuity
  last_tau_ = tau;
  last_tau_valid_ = true;

  state_cycles_++;

  return tau;
}

// Implement additional pure virtuals from Controller
void PegInHoleFSM::setQInit(const tum_ics_ur_robot_lli::JointState& qinit)
{
  q_init_ = qinit;

  // Propagate to sub-controllers (only joint_ctrl_ and cartesian_ctrl_)
  joint_ctrl_.setQInit(qinit);
  cartesian_ctrl_.setQInit(qinit);

  ROS_INFO_STREAM("PegInHoleFSM::setQInit() - stored and propagated to sub-controllers");
}

void PegInHoleFSM::setQHome(const tum_ics_ur_robot_lli::JointState& qhome)
{
  q_home_stored_ = qhome;

  // Propagate to sub-controllers (only joint_ctrl_ and cartesian_ctrl_)
  joint_ctrl_.setQHome(qhome);
  cartesian_ctrl_.setQHome(qhome);

  // Use YAML q_safe if provided; otherwise fall back to robot qhome
  if (!has_q_safe_param_)
  {
    q_safe_ = qhome.q;
  }

  ROS_INFO_STREAM("PegInHoleFSM::setQHome() - stored and propagated to sub-controllers");
}

void PegInHoleFSM::setQPark(const tum_ics_ur_robot_lli::JointState& qpark)
{
  q_park_ = qpark;

  // Propagate to sub-controllers (only joint_ctrl_ and cartesian_ctrl_)
  joint_ctrl_.setQPark(qpark);
  cartesian_ctrl_.setQPark(qpark);

  ROS_INFO_STREAM("PegInHoleFSM::setQPark() - stored and propagated to sub-controllers");
}

} // namespace tum_ics_ur10_controller_tutorial
