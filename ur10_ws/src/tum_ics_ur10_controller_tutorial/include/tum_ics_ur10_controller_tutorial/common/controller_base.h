#pragma once

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tum_ics_ur_robot_lli/JointState.h>
#include <tum_ics_ur_robot_lli/RobotTime.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Base class for all controllers in the peg-in-hole system
 *
 * Provides a unified interface for joint space, cartesian space,
 * gripper, and FSM controllers. Inherits from the LLI ControlEffort
 * for integration with the robot control framework.
 *
 * Lifecycle:
 *   init()  -> start() -> update() -> stop()
 *
 * Pure virtual - no control law implemented here.
 */
class ControllerBase
  : public tum_ics_ur_robot_lli::RobotControllers::ControlEffort
{
public:
  /**
   * @brief Constructor
   * @param name Controller name for logging
   * @param type Control type (from ControlEffort)
   * @param space Control space (from ControlEffort)
   * @param weight Controller weight for blending (default 1.0)
   */
  ControllerBase(
    const QString& name = "ControllerBase",
    tum_ics_ur_robot_lli::RobotControllers::ControlType type = tum_ics_ur_robot_lli::RobotControllers::STANDARD_TYPE,
    tum_ics_ur_robot_lli::RobotControllers::ControlSpace space = tum_ics_ur_robot_lli::RobotControllers::JOINT_SPACE,
    double weight = 1.0)
    : ControlEffort(name, type, space, weight)
  {}

  virtual ~ControllerBase() = default;

  // Optional FSM lifecycle hooks (no-op by default)
  virtual void onEnterState(const tum_ics_ur_robot_lli::RobotTime&,
                            const tum_ics_ur_robot_lli::JointState&)
  {}

  virtual void onExitState() {}

  // Public wrappers for FSM to call protected methods
  bool callInit() { return init(); }
  bool callStart() { return start(); }
  Tum::VectorDOFd callUpdate(const tum_ics_ur_robot_lli::RobotTime& time, const tum_ics_ur_robot_lli::JointState& state) { return update(time, state); }
  bool callStop() { return stop(); }

protected:
  /**
   * @brief Initialize controller
   *
   * Called once before start(). Use this to:
   * - Allocate gain matrices
   * - Read ROS parameters
   * - Initialize state variables
   *
   * @return true if initialization successful
   */
  virtual bool init() override = 0;

  /**
   * @brief Start controller
   *
   * Called when controller becomes active. Use this to:
   * - Reset integrators
   * - Reset internal timers
   * - Capture initial state
   *
   * @return true if start successful
   */
  virtual bool start() override = 0;

  /**
   * @brief Update controller (compute control command)
   *
   * Called at every control cycle. Implement your control law here.
   *
   * @param time Current robot time
   * @param state Current joint state (from LLI JointState)
   * @return Joint torque command (Tum::VectorDOFd)
   */
  virtual Tum::VectorDOFd update(
    const tum_ics_ur_robot_lli::RobotTime& time,
    const tum_ics_ur_robot_lli::JointState& state
  ) override = 0;

  /**
   * @brief Stop controller
   *
   * Called when controller becomes inactive. Use this to:
   * - Safe shutdown
   * - Save final state if needed
   *
   * @return true if stop successful
   */
  virtual bool stop() override = 0;

  /**
   * @brief Set initial joint state
   * @param qinit Initial joint configuration
   */
  virtual void setQInit(const tum_ics_ur_robot_lli::JointState& qinit) override = 0;

  /**
   * @brief Set home joint state
   * @param qhome Home joint configuration
   */
  virtual void setQHome(const tum_ics_ur_robot_lli::JointState& qhome) override = 0;

  /**
   * @brief Set park joint state
   * @param qpark Park joint configuration
   */
  virtual void setQPark(const tum_ics_ur_robot_lli::JointState& qpark) override = 0;

};

} // namespace tum_ics_ur10_controller_tutorial
