#pragma once

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tum_ics_ur10_controller_tutorial/common/types.h>
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
   * @param weight Controller weight for blending (default 1.0)
   * @param name Controller name for logging
   */
  ControllerBase(double weight = 1.0, const QString& name = "ControllerBase")
    : ControlEffort(weight, name)
  {}

  virtual ~ControllerBase() = default;

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
   * @return Joint torque command (Eigen::VectorXd)
   */
  virtual Eigen::VectorXd update(
    const RobotTime& time,
    const JointState& state
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

protected:
  // Note: robot_ pointer is provided by ControlEffort base class
  // Available methods:
  //   robot_->getJacobian()          - Get Jacobian matrix
  //   robot_->getEndEffectorPose()   - Get current EE pose (Eigen::Affine3d)
  //   robot_->qHome()                - Get home position
  //   robot_->qPark()                - Get park position
  //   robot_->getDOF()               - Get degrees of freedom
  //   robot_->getTime()              - Get current time
};

} // namespace tum_ics_ur10_controller_tutorial
