#include <QCoreApplication>
#include <QDebug>

#include <ros/ros.h>
#include <tum_ics_lacquey_gripper_driver/GripperDriver.h>

#include <Common/ConsoleReader.h>

using namespace tum_ics_lacquey_gripper_driver;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"tum_ics_lacquey_gripper_driver",ros::init_options::AnonymousName);
    QCoreApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(30);

    Tum::Tools::Common::ConsoleReader console;

    if(argc < 5)
    {
        ROS_ERROR("Invalid number of arguments: argc = %d",argc);
        return -1;
    }

    QString gripperTypeStr = QString(argv[1]);
    QString pcIpAddr = QString(argv[2]);
    QString robotIpAddr = QString(argv[3]);
    QString gripperPortStr = QString(argv[4]);

    QString gripperPrefix;

    //If no prefix is passed then we use ""
    if(argc == 6)
        gripperPrefix= QString(argv[5]);
    else
        gripperPrefix="";

    Gripper::Type gripperType;

    if(gripperTypeStr == "right")
    {
        gripperType = Gripper::RIGHT_GRIPPER;
    }
    else if(gripperTypeStr == "left")
    {
        gripperType = Gripper::LEFT_GRIPPER;
    }else if(gripperTypeStr == "std")
    {
        gripperType = Gripper::STANDARD_GRIPPER;
    }
    else
    {
        ROS_ERROR("Invalid gripper type '%s'",gripperTypeStr.toLatin1().data());
        return -1;
    }

    QHostAddress ip;
    if(!ip.setAddress(pcIpAddr))
    {
        ROS_ERROR("Invalid pc ip address '%s'",pcIpAddr.toLatin1().data());
        return -1;
    }

    if(!ip.setAddress(robotIpAddr))
    {
        ROS_ERROR("Invalid robot ip address '%s'",robotIpAddr.toLatin1().data());
        return -1;
    }

    bool ok;
    int gripperPort = gripperPortStr.toUShort(&ok);
    if(!ok)
    {
        ROS_ERROR("Invalid gripper port '%s'",gripperPortStr.toLatin1().data());
        return -1;
    }

    ROS_INFO("gripper:       '%s'", gripperTypeStr.toLatin1().data());
    ROS_INFO("pc:             %s", pcIpAddr.toLatin1().data());
    ROS_INFO("robot:          %s", robotIpAddr.toLatin1().data());
    ROS_INFO("gripper port:   %d", gripperPort);
    ROS_INFO("gripper type:   %d", gripperType);
    ROS_INFO_STREAM("gripper prefix: "<<gripperPrefix.toStdString());

//    Gripper::JointStatePub testJointState(Gripper::STANDARD_GRIPPER,"/ursa_fingers_joint_states","ursa_");


    //Get the realRobot from RobotArm (when robot arm is exectuted it will read the *.ini file and get real/sim parameter)
    //We need to check the namespace of the node that instatiates robotArm.
    bool realGripper = true;

    std::string param="realRobot";

    if(!ros::param::has(param))
    {
        ROS_ERROR("Parameter '%s' is missing.", param.c_str());
        return false;
    }

    if(!ros::param::get(param,realGripper))
    {
        ROS_ERROR("Couldn't get parameter '%s'.", param.c_str());
        return false;
    }
    //ros::param::get("~realRobot",realGripper);

    ROS_WARN_STREAM("Real Robot: "<<realGripper);


    Gripper::GripperDriver gripper(gripperType,pcIpAddr,robotIpAddr,gripperPort,realGripper,gripperPrefix);

    if(!gripper.start())
    {
        ROS_ERROR_STREAM("Lacquey Gripper ERROR: The gripper couldn't start");
        return -1;
    }

//    int cnt=0;
    bool flag = true;
    QString line;
    bool hasLine;

    while(ros::ok())
    {
        gripper.update();
        gripper.publish();

        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            gripper.stop();
            ros::shutdown();
        }

        if(hasLine && (line == "open"))
        {
            gripper.send("open");
        }

        if(hasLine && (line == "close"))
        {
            gripper.send("close");
        }

        if(hasLine && (line == "free"))
        {
            gripper.send("free");
        }

        QCoreApplication::processEvents();
        ros::spinOnce();
        r.sleep();
//        cnt++;
    }

    return 0;

}

