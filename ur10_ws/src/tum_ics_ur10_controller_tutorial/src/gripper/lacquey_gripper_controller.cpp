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
    state_topic_("/lacquey_gripper/state"),
    service_name_("/setGripperState"),
    use_service_(false)
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
  nh.param("gripper/service_name", service_name_, service_name_);
  nh.param("gripper/use_service", use_service_, true);

  // Setup publishers
  width_cmd_pub_ = nh.advertise<std_msgs::Float64>(width_topic_, 1);
  force_cmd_pub_ = nh.advertise<std_msgs::Float64>(force_topic_, 1);

  // Setup subscriber
  state_sub_ = nh.subscribe(state_topic_, 1,
                            &LacqueyGripperController::stateCallback, this);

  // Setup service client (optional)
  if (use_service_)
  {
    set_state_client_ = nh.serviceClient<tum_ics_lacquey_gripper_msgs::setGripperState>(service_name_);
    if (!set_state_client_.waitForExistence(ros::Duration(1.0)))
    {
      ROS_WARN_STREAM("Gripper service not available: " << service_name_
                      << ". Falling back to topic commands.");
      use_service_ = false;
    }
  }

  ROS_INFO_STREAM("Gripper interface initialized:");
  ROS_INFO_STREAM("  Open width: " << open_width_ << " m");
  ROS_INFO_STREAM("  Close width: " << close_width_ << " m");
  ROS_INFO_STREAM("  Grasp force: " << grasp_force_ << " N");
  ROS_INFO_STREAM("  Width topic: " << width_topic_);
  ROS_INFO_STREAM("  Force topic: " << force_topic_);
  ROS_INFO_STREAM("  State topic: " << state_topic_);
  ROS_INFO_STREAM("  Service: " << service_name_ << " (use_service=" << (use_service_ ? "true" : "false") << ")");

  return true;
}

void LacqueyGripperController::open()
{
  target_width_ = open_width_;

  if (use_service_)
  {
    sendStateCommand("open");
    current_width_ = target_width_;
    return;
  }

  std_msgs::Float64 msg;
  msg.data = target_width_;
  width_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper opening to " << target_width_ << " m");
}

void LacqueyGripperController::close()
{
  target_width_ = close_width_;

  if (use_service_)
  {
    sendStateCommand("close");
    current_width_ = target_width_;
    return;
  }

  std_msgs::Float64 msg;
  msg.data = target_width_;
  width_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper closing to " << target_width_ << " m");
}

void LacqueyGripperController::setWidth(double width)
{
  // Clamp to valid range
  target_width_ = std::max(close_width_, std::min(width, open_width_));

  if (use_service_)
  {
    // Map to open/close for service-based driver
    const double mid = (open_width_ + close_width_) * 0.5;
    sendStateCommand((target_width_ >= mid) ? "open" : "close");
    current_width_ = target_width_;
    return;
  }

  std_msgs::Float64 msg;
  msg.data = target_width_;
  width_cmd_pub_.publish(msg);

  ROS_INFO_STREAM("Gripper width set to " << target_width_ << " m");
}

void LacqueyGripperController::grasp(double force)
{
  if (use_service_)
  {
    (void)force; // force is not supported by service driver
    sendStateCommand("close");
    current_width_ = close_width_;
    return;
  }

  // Publish force command (topic-based driver)
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

bool LacqueyGripperController::sendStateCommand(const std::string& cmd)
{
  if (!set_state_client_.exists())
  {
    ROS_WARN_STREAM("Gripper service not available: " << service_name_);
    return false;
  }

  tum_ics_lacquey_gripper_msgs::setGripperState srv;
  srv.request.newState = cmd;
  if (!set_state_client_.call(srv) || !srv.response.ok)
  {
    ROS_WARN_STREAM("Failed to set gripper state to '" << cmd << "'");
    return false;
  }

  ROS_INFO_STREAM("Gripper state set to '" << cmd << "'");
  return true;
}

} // namespace tum_ics_ur10_controller_tutorial
