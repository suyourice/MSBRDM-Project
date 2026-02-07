#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <tum_ics_ur10_controller_tutorial/joint/joint_pd_controller.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  ros::init(argc, argv, "joint_pd_test_node");

  if (argc < 2)
  {
    ROS_ERROR("Usage: joint_pd_test_node <robot_config_file>");
    return -1;
  }

  QString configFilePath = argv[1];
  ROS_INFO_STREAM("Loading robot config from: " << configFilePath.toStdString());

  // Create robot interface
  tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);

  if (!robot.init())
  {
    ROS_ERROR("Failed to initialize robot");
    return -1;
  }

  // Create joint PD controller
  tum_ics_ur10_controller_tutorial::JointPDController controller(1.0);

  // Add controller to robot
  robot.add(&controller);

  // Start robot control
  ROS_INFO("Starting robot control...");
  robot.start();

  ROS_INFO("Joint PD controller running. Moving to HOME position.");
  ROS_INFO("Press Ctrl+C to stop.");

  // Spin
  ros::spin();

  // Stop robot
  ROS_INFO("Stopping robot...");
  robot.stop();

  return 0;
}
