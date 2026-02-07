#include <tum_ics_ur10_controller_tutorial/gripper/lacquey_gripper_controller.h>

namespace tum_ics_ur10_controller_tutorial
{

LacqueyGripperController::LacqueyGripperController()
  : open_width_(0.08),
    close_width_(0.0),
    grasp_force_(20.0),
    current_width_(0.0),
    target_width_(0.0),
    width_topic_("/lacquey_gripper/width_cmd"),
    force_topic_("/lacquey_gripper/force_cmd"),
    state_topic_("/lacquey_gripper/state")
{
}

bool LacqueyGripperController::init(ros::NodeHandle& nh)
{
  ROS_INFO("LacqueyGripperController::init()");

  // Read parameters
  nh.param("gripper/open_width", open_width_, 0.08);
  nh.param("gripper/close_width", close_width_, 0.0);
  nh.param("gripper/grasp_force", grasp_force_, 20.0);
  nh.param("gripper/width_topic", width_topic_, width_topic_);
  nh.param("gripper/force_topic", force_topic_, force_topic_);
  nh.param("gripper/state_topic", state_topic_, state_topic_);

  // Setup publishers
  width_cmd_pub_ = nh.advertise<std_msgs::Float64>(width_topic_, 1);
  force_cmd_pub_ = nh.advertise<std_msgs::Float64>(force_topic_, 1);

  // Setup subscriber
  state_sub_ = nh.subscribe(state_topic_, 1,
                            &LacqueyGripperController::stateCallback, this);

  ROS_INFO_STREAM("Gripper interface initialized:");
  ROS_INFO_STREAM("  Open width: " << open_width_ << " m");
  ROS_INFO_STREAM("  Close width: " << close_width_ << " m");
  ROS_INFO_STREAM("  Grasp force: " << grasp_force_ << " N");
  ROS_INFO_STREAM("  Width topic: " << width_topic_);
  ROS_INFO_STREAM("  Force topic: " << force_topic_);
  ROS_INFO_STREAM("  State topic: " << state_topic_);

  return true;
}

void LacqueyGripperController::open()
{
  target_width_ = open_width_;

  std_msgs::Float64 msg;
  msg.data = target_width_;
  width_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper opening to " << target_width_ << " m");
}

void LacqueyGripperController::close()
{
  target_width_ = close_width_;

  std_msgs::Float64 msg;
  msg.data = target_width_;
  width_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper closing to " << target_width_ << " m");
}

void LacqueyGripperController::setWidth(double width)
{
  // Clamp to valid range
  target_width_ = std::max(close_width_, std::min(width, open_width_));

  std_msgs::Float64 msg;
  msg.data = target_width_;
  width_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper width set to " << target_width_ << " m");
}

void LacqueyGripperController::grasp(double force)
{
  // Publish force command
  std_msgs::Float64 msg;
  msg.data = force;
  force_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper grasping with force " << force << " N");
}

bool LacqueyGripperController::isAtTarget(double tolerance) const
{
  return std::abs(current_width_ - target_width_) < tolerance;
}

void LacqueyGripperController::stateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  current_width_ = msg->data;
}

} // namespace tum_ics_ur10_controller_tutorial
