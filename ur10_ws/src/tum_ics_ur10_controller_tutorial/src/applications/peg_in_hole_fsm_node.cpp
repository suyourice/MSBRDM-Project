#include <tum_ics_ur10_controller_tutorial/fsm/peg_in_hole_fsm.h>
#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <ros/ros.h>
#include <QApplication>

int main(int argc, char** argv)
{
  // Initialize Qt application (required by RobotArm)
  QApplication app(argc, argv);

  // Initialize ROS
  ros::init(argc, argv, "peg_in_hole_fsm_node", ros::init_options::AnonymousName);

  ROS_INFO("Starting Peg-in-Hole FSM Controller");

  // Get config file path from launch file argument
  if (argc < 2)
  {
    ROS_ERROR("Usage: peg_in_hole_fsm_node <config_file_path>");
    ROS_ERROR("Config file should be configUR10.ini");
    return -1;
  }

  QString configFilePath = argv[1];
  ROS_INFO_STREAM("Config File: " << configFilePath.toStdString());

  // Create robot interface
  ROS_INFO("Creating RobotArm");
  tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);
  if (!robot.init())
  {
    ROS_ERROR("Failed to initialize robot");
    return -1;
  }

  // Create FSM controller
  ROS_INFO("Creating PegInHoleFSM controller");
  tum_ics_ur10_controller_tutorial::PegInHoleFSM fsm(1.0, "PegInHoleFSM");

  // Add controller to robot (this connects them)
  ROS_INFO("Adding controller to robot");
  if (!robot.add(&fsm))
  {
    ROS_ERROR("Failed to add FSM controller to robot");
    return -1;
  }

  // Set home/park positions
  ROS_INFO("Setting home and park positions");
  fsm.setQHome(robot.qHome());
  fsm.setQPark(robot.qPark());

  // Optional startup delay for visualization setup
  ros::NodeHandle pnh("~");
  double startup_delay = 0.0;
  pnh.param("startup_delay", startup_delay, 0.0);
  if (startup_delay > 0.0)
  {
    ROS_INFO_STREAM("Startup delay: waiting " << startup_delay << "s before starting FSM");
    ros::Duration(startup_delay).sleep();
  }

  // Explicitly start FSM (RobotArm does not always call controller::start)
  ROS_INFO("Starting PegInHoleFSM controller");
  if (!fsm.callStart())
  {
    ROS_ERROR("Failed to start PegInHoleFSM controller");
    return -1;
  }

  // Start robot (this starts the control loop)
  ROS_INFO("Starting robot");
  robot.start();

  ROS_INFO("======================================");
  ROS_INFO("Peg-in-Hole FSM started successfully!");
  ROS_INFO("Press Ctrl+C to stop");
  ROS_INFO("======================================");

  // Spin ROS
  ros::spin();

  // Stop robot
  ROS_INFO("Stopping robot");
  robot.stop();

  ROS_INFO("Peg-in-Hole FSM stopped");

  return 0;
}
