#include <tum_ics_ur10_controller_tutorial/fsm/peg_in_hole_fsm.h>
#include <tum_ics_ur_robot_lli/robot_lli.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "peg_in_hole_fsm_node");
  ros::NodeHandle nh("~");

  ROS_INFO("Starting Peg-in-Hole FSM Controller");

  // Create robot interface
  auto robot = std::make_shared<tum_ics_ur_robot_lli::RobotLLI>();

  // Create FSM controller
  auto fsm = std::make_shared<tum_ics_ur10_controller_tutorial::PegInHoleFSM>(
      1.0, "PegInHoleFSM");

  // Set robot for controller
  fsm->setRobot(robot);

  // Initialize
  if (!robot->init())
  {
    ROS_ERROR("Failed to initialize robot");
    return -1;
  }

  if (!fsm->init())
  {
    ROS_ERROR("Failed to initialize FSM");
    return -1;
  }

  // Start robot
  if (!robot->start())
  {
    ROS_ERROR("Failed to start robot");
    return -1;
  }

  // Start controller
  if (!fsm->start())
  {
    ROS_ERROR("Failed to start FSM");
    return -1;
  }

  ROS_INFO("Peg-in-Hole FSM started successfully");
  ROS_INFO("Press Ctrl+C to stop");

  // Spin
  ros::spin();

  // Stop
  fsm->stop();
  robot->stop();

  ROS_INFO("Peg-in-Hole FSM stopped");

  return 0;
}
