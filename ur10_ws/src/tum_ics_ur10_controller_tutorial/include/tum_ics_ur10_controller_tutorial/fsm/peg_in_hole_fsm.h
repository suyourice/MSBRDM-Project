#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur10_controller_tutorial/joint/joint_pd_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_pdg_controller.h>
#include <tum_ics_ur10_controller_tutorial/gripper/lacquey_gripper_controller.h>
#include <ros/ros.h>
#include <Eigen/Dense>

// Removed: #include <tum_ics_ur10_controller_tutorial/cartesian/hybrid_insertion_controller.h>
// Reason: CIRCULAR_INSERTION now handled by cartesian_pdg_controller + FSM trajectory

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Finite State Machine for peg-in-hole task
 *
 * Orchestrates all controllers to perform complete peg-in-hole assembly:
 *   HOME → OPEN_GRIPPER → MOVE_TO_PEG → GRASP_PEG → LIFT_PEG →
 *   MOVE_ABOVE_HOLE → ALIGN_ORIENTATION → CIRCULAR_INSERTION →
 *   SUCCESS/RETRY → DONE
 *
 * Uses:
 * - JointPDController: HOME positioning
 * - CartesianPDGController: Waypoint navigation
 * - HybridInsertionController: Peg insertion
 * - LacqueyGripperController: Grasping
 */
class PegInHoleFSM : public ControllerBase
{
public:
  enum class State
  {
    INIT,                 // Initialize controllers
    MOVE_TO_SAFE,         // Move to safe position (joint control with interpolation)
    OPEN_GRIPPER,         // Open gripper
    HOLD_CARTESIAN,       // Hold current pose before switching to Cartesian motion
    MOVE_TO_PEG,          // Approach peg location (Cartesian)
    DESCEND_TO_PEG,       // Descend to grasp height
    CLOSE_GRIPPER,        // Grasp peg
    LIFT_PEG,             // Lift peg up
    MOVE_ABOVE_HOLE,      // Move above hole (ArUco vision)
    ALIGN_ORIENTATION,    // Align peg with hole
    CIRCULAR_INSERTION,   // Insert with spiral search + compliance
    SUCCESS,              // Insertion complete
    RETRY,                // Retry insertion
    DONE                  // Task complete
  };

  PegInHoleFSM(double weight = 1.0, const QString& name = "PegInHoleFSM");

  virtual ~PegInHoleFSM() = default;

  /**
   * @brief Get current state
   */
  State getCurrentState() const { return state_; }

  /**
   * @brief Get state name as string
   */
  std::string getStateName() const;

  // Additional pure virtuals from Controller base (PUBLIC for RobotArm to call)
  void setQInit(const tum_ics_ur_robot_lli::JointState& qinit) override;
  void setQHome(const tum_ics_ur_robot_lli::JointState& qhome) override;
  void setQPark(const tum_ics_ur_robot_lli::JointState& qpark) override;

protected:
  bool init() override;
  bool start() override;
  Tum::VectorDOFd update(const tum_ics_ur_robot_lli::RobotTime& time,
                         const tum_ics_ur_robot_lli::JointState& state) override;
  bool stop() override;

private:
  // State transition logic
  void switchState(State new_state,
                   const tum_ics_ur_robot_lli::RobotTime& time,
                   const tum_ics_ur_robot_lli::JointState& state);
  bool checkStateComplete(const tum_ics_ur_robot_lli::RobotTime& time);
  void applyStateTargets(const tum_ics_ur_robot_lli::JointState& state);

  // Sub-controllers
  // CRITICAL: Only TWO controllers for entire task!
  // - joint_ctrl_: Used only for INIT + MOVE_TO_SAFE + OPEN_GRIPPER
  // - cartesian_ctrl_: Used from HOLD_CARTESIAN until DONE (single instance!)
  JointPDController joint_ctrl_;
  CartesianPDGController cartesian_ctrl_;
  LacqueyGripperController gripper_;

  // Removed: HybridInsertionController insertion_ctrl_;
  // Reason: CIRCULAR_INSERTION now handled by cartesian_ctrl_ + FSM trajectory generation

  // Current state
  State state_;
  State pending_transition_;
  bool has_pending_transition_;
  int state_cycles_;
  bool target_pending_;
  tum_ics_ur_robot_lli::RobotTime state_start_time_;
  tum_ics_ur_robot_lli::RobotTime last_time_;

  // Torque continuity (hold during transitions)
  Eigen::VectorXd last_tau_;
  bool last_tau_valid_;

  // Stored robot configurations
  tum_ics_ur_robot_lli::JointState q_init_;
  tum_ics_ur_robot_lli::JointState q_home_stored_;
  tum_ics_ur_robot_lli::JointState q_park_;

  // State timeouts
  std::map<State, double> state_timeouts_;
  double min_dwell_joint_to_cartesian_;
  double min_dwell_cartesian_;
  double hold_cartesian_time_;

  // Waypoints (from config)
  Eigen::VectorXd q_safe_;          // User-defined safe position
  Eigen::Vector3d p_peg_;
  Eigen::Vector3d p_hole_;
  Eigen::Matrix3d R_peg_;
  bool has_q_safe_param_;
  bool use_frame_transform_;
  Eigen::Matrix3d frame_R_;
  Eigen::Vector3d frame_t_;
  bool use_tool_transform_;
  Eigen::Affine3d tool_T_;

  // Retry logic
  int retry_count_;
  int max_retries_;

  // Circular insertion trajectory generation (FSM-managed)
  Eigen::Vector3d insertion_start_pos_;  // Starting position for insertion
  double insertion_radius_;              // Circular search radius (m)
  double insertion_omega_;               // Angular velocity (rad/s)
  double insertion_v_descent_;           // Descent velocity (m/s)
  double insertion_target_depth_;        // Target insertion depth (m)
  double insertion_start_time_;          // Time when insertion started
  bool insertion_success_;               // Success flag

  // ROS
  ros::NodeHandle nh_;

  // Flags
  bool use_aruco_;  // Use ArUco for hole detection
};

} // namespace tum_ics_ur10_controller_tutorial
