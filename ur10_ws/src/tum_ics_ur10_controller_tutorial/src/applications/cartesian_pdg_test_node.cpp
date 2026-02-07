#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <tum_ics_ur10_controller_tutorial/cartesian/cartesian_pdg_controller.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  ros::init(argc, argv, "cartesian_pdg_test_node");

  if (argc < 2)
  {
    ROS_ERROR("Usage: cartesian_pdg_test_node <robot_config_file>");
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

  // Create Cartesian PDG controller
  tum_ics_ur10_controller_tutorial::CartesianPDGController controller(1.0);

  // Add controller to robot
  robot.add(&controller);

  // Start robot control
  ROS_INFO("Starting robot control...");
  robot.start();

  // Wait for controller to initialize
  ros::Duration(1.0).sleep();

  // Set a test target: move 10cm in +Z direction
  Eigen::Affine3d target_pose = robot.getEndEffectorPose();
  target_pose.translation().z() += 0.1;  // Move up 10cm

  ROS_INFO_STREAM("Current position: " << robot.getEndEffectorPose().translation().transpose());
  ROS_INFO_STREAM("Target position: " << target_pose.translation().transpose());

  controller.setDesiredPose(target_pose);

  ROS_INFO("Cartesian PDG controller running. Moving to target.");
  ROS_INFO("Press Ctrl+C to stop.");

  // Spin
  ros::spin();

  // Stop robot
  ROS_INFO("Stopping robot...");
  robot.stop();

  return 0;
}
