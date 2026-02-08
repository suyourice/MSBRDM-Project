#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tum_ics_lacquey_gripper_msgs/setGripperState.h>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Simple interface for Lacquey gripper control
 *
 * Provides basic open/close/grasp commands via ROS topics.
 * This is a simple wrapper, not a full controller inheriting from ControllerBase.
 */
class LacqueyGripperController
{
public:
  LacqueyGripperController();

  virtual ~LacqueyGripperController() = default;

  /**
   * @brief Initialize gripper interface
   * @param nh ROS node handle
   * @return true if successful
   */
  bool init(ros::NodeHandle& nh);

  /**
   * @brief Open gripper to maximum width
   */
  void open();

  /**
   * @brief Close gripper completely
   */
  void close();

  /**
   * @brief Set gripper to specific width
   * @param width Target width in meters
   */
  void setWidth(double width);

  /**
   * @brief Grasp with specific force
   * @param force Target grasping force in Newtons
   */
  void grasp(double force);

  /**
   * @brief Get current gripper state
   * @return Current width (m)
   */
  double getCurrentWidth() const { return current_width_; }

  /**
   * @brief Check if gripper is at target
   * @param tolerance Width tolerance (m)
   */
  bool isAtTarget(double tolerance = 0.005) const;

private:
  // ROS communication
  ros::Publisher width_cmd_pub_;
  ros::Publisher force_cmd_pub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_state_client_;

  // State callback
  void stateCallback(const std_msgs::Float64::ConstPtr& msg);
  bool sendStateCommand(const std::string& cmd);

  // Configuration
  double open_width_;    // Maximum open width (m)
  double close_width_;   // Minimum close width (m)
  double grasp_force_;   // Default grasping force (N)

  // Current state
  double current_width_;
  double target_width_;

  // Topic names
  std::string width_topic_;
  std::string force_topic_;
  std::string state_topic_;
  std::string service_name_;

  bool use_service_;
};

} // namespace tum_ics_ur10_controller_tutorial
