#ifndef LACQUEY_GRIPPER_DRIVER_GRIPPER_GRIPPERDRIVER_H
#define LACQUEY_GRIPPER_DRIVER_GRIPPER_GRIPPERDRIVER_H

#include <QString>
#include <QTcpSocket>
#include <QTcpServer>
#include <QElapsedTimer>

#include <ros/ros.h>
#include <tum_ics_lacquey_gripper_driver/Defs.h>
#include <tum_ics_lacquey_gripper_driver/JointStatePub.h>

#include <tum_ics_ur_robot_msgs/setScriptManagerState.h>
#include <tum_ics_ur_robot_msgs/getScriptManagerStates.h>

#include <tum_ics_lacquey_gripper_msgs/setGripperState.h>
#include <tum_ics_lacquey_gripper_msgs/getGripperState.h>

namespace tum_ics_lacquey_gripper_driver{
namespace Gripper{

class GripperDriver
{
public:
    static QString defaultRobotIpAddr(Type t);
    static QVector<QString> commands();

    enum Command
    {
        ACTIVATE_GRIPPER_CMD    = 20,
        DEACTIVATE_GRIPPER_CMD  = 21,
        OPEN_GRIPPER_CMD        = 22,
        CLOSE_GRIPPER_CMD       = 23
    };

private:
    ros::NodeHandle     m_node;
    ros::ServiceClient  m_setScriptState;
    ros::ServiceServer  m_setState;
    ros::ServiceServer  m_getState;
    ros::Publisher      m_statePub;

    JointStatePub m_jointStatePub;

    Type m_type;

    QTcpServer* m_server;
    QTcpSocket* m_socket;

    QString m_pcIp;
    QString m_robotIp;
    quint16 m_gripperPort;

    QString m_state;
    QString m_pendingState;
    QString m_lastState;
    QElapsedTimer m_timer;
    QString m_prefix;
    bool m_realGripper;

protected:
    bool m_started;

public:
    // empty strings == default values
    GripperDriver(Type type = RIGHT_GRIPPER,
            const QString &pcIpAddr = "192.168.1.3",
            const QString &robotIpAddr = QString(),
            quint16 gripperPort = 50003,
            const bool realGripper = true,
            const QString& prefixIn = "",
            const QString& jointStateTopicName = QString());

    ~GripperDriver();

    bool start();
    void stop();

    bool send(const QString& cmd);

    void update();

    // publish joint state topic
    void publish();

private:
    bool enableScript(bool enable);
    bool send(Command cmd);

    bool setStateCallback(
            tum_ics_lacquey_gripper_msgs::setGripperState::Request &req,
            tum_ics_lacquey_gripper_msgs::setGripperState::Response &res);

    bool getStateCallback(
            tum_ics_lacquey_gripper_msgs::getGripperState::Request &req,
            tum_ics_lacquey_gripper_msgs::getGripperState::Response &res);

    bool getParamString(QString& str, const QString& param);
    bool hasParam(const QString& param);

};

}}

#endif // LACQUEY_GRIPPER_DRIVER_GRIPPER_GRIPPERDRIVER_H
