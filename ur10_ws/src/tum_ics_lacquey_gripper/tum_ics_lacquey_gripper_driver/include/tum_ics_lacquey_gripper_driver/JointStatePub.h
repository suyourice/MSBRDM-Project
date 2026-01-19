#ifndef LACQUEY_GRIPPER_DRIVER_GRIPPER_JOINT_STATE_PUB_H
#define LACQUEY_GRIPPER_DRIVER_GRIPPER_JOINT_STATE_PUB_H

#include <ros/ros.h>

#include <tum_ics_lacquey_gripper_driver/Defs.h>
#include <tum_ics_lacquey_gripper_driver/JointState.h>

#include <QString>

namespace tum_ics_lacquey_gripper_driver{
namespace Gripper{

class JointStatePub
{
public:
    static QVector<JointState> fingerJoints(Type t, State s, const QString& prefix = "");

private:
    static QString typePrefix(Type t, const QString& prefixIn = "");
    static double statePosJoint0(State s);
    static double statePosJoint1(State s);

private:
    ros::NodeHandle     m_node;
    ros::Publisher      m_pub;

    QString m_topicName;
    QString m_prefix; //* prefix used to construct the topic name and the joint_state names
    Type m_type;

    QVector<JointState> m_joints;

public:
    // empty topic name: use default
    JointStatePub(
            Type type = LEFT_GRIPPER,
            const QString& topicName = "",
            const QString& prefix = "");

    ~JointStatePub();

    void setState(State s);

    void publish();

};

}}

#endif // LACQUEY_GRIPPER_DRIVER_GRIPPER_JOINT_STATE_PUB_H
