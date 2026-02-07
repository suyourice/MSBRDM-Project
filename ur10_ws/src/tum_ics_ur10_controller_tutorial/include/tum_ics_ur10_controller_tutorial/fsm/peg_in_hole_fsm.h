#pragma once

#include <tum_ics_ur10_controller_tutorial/common/controller_base.h>
#include <tum_ics_ur10_controller_tutorial/joint/joint_pd_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_pdg_controller.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/hybrid_insertion_controller.h>
#include <tum_ics_ur10_controller_tutorial/gripper/lacquey_gripper_controller.h>
#include <ros/ros.h>
#include <Eigen/Dense>

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
    HOME,                 // Move to home position (joint control)
    OPEN_GRIPPER,         // Open gripper
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

protected:
  bool init() override;
  bool start() override;
  Tum::VectorDOFd update(const tum_ics_ur_robot_lli::RobotTime& time,
                         const tum_ics_ur_robot_lli::JointState& state) override;
  bool stop() override;

  // Additional pure virtuals from Controller base
  void setQInit(const tum_ics_ur_robot_lli::JointState& qinit) override;
  void setQHome(const tum_ics_ur_robot_lli::JointState& qhome) override;
  void setQPark(const tum_ics_ur_robot_lli::JointState& qpark) override;

private:
  // State transition logic
  void transitionTo(State new_state);
  bool checkStateComplete();

  // Sub-controllers
  JointPDController joint_ctrl_;
  CartesianPDGController cartesian_ctrl_;
  HybridInsertionController insertion_ctrl_;
  LacqueyGripperController gripper_;

  // Current state
  State state_;
  tum_ics_ur_robot_lli::RobotTime state_start_time_;
  tum_ics_ur_robot_lli::RobotTime last_time_;

  // Stored robot configurations
  tum_ics_ur_robot_lli::JointState q_init_;
  tum_ics_ur_robot_lli::JointState q_home_stored_;
  tum_ics_ur_robot_lli::JointState q_park_;

  // State timeouts
  std::map<State, double> state_timeouts_;

  // Waypoints (from config)
  Eigen::VectorXd q_home_;
  Eigen::Vector3d p_peg_;
  Eigen::Vector3d p_hole_;
  Eigen::Matrix3d R_peg_;

  // Retry logic
  int retry_count_;
  int max_retries_;

  // ROS
  ros::NodeHandle nh_;

  // Flags
  bool use_aruco_;  // Use ArUco for hole detection
};

} // namespace tum_ics_ur10_controller_tutorial
