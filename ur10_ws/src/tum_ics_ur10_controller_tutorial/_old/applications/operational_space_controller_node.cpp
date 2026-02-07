#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <tum_ics_ur10_controller_tutorial/operational_space_controller.h>
#include <QApplication>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  ros::init(argc, argv, "ur10_operational_controller");

  QString configFilePath = argv[1];
  tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);

  if (!robot.init())
    return -1;

  tum_ics_ur10_controller_tutorial::OperationalSpaceController ctrl(1.0);

  robot.add(&ctrl);

  robot.start();
  ros::spin();
  robot.stop();

  return 0;
}
