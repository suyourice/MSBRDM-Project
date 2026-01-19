#include <tum_ics_lacquey_gripper_driver/JointStatePub.h>

#include <math.h>

namespace tum_ics_lacquey_gripper_driver{
namespace Gripper{

QVector<JointState> JointStatePub::fingerJoints(Type t, State s, const QString& prefix)
{
    QVector<JointState> joints;

    const QVector<QString> fingers = QVector<QString>()
            << "left_finger"
            << "right_finger"
            << "mid_finger";

    for(int i=0; i<fingers.size(); i++)
    {
        joints << JointState(typePrefix(t,prefix)+fingers.at(i)+"_base_joint",statePosJoint0(s));
        joints << JointState(typePrefix(t,prefix)+fingers.at(i)+"_joint",statePosJoint1(s));
    }

    return joints;
}

QString JointStatePub::typePrefix(Type t, const QString& prefixIn)
{
    QString prefix;

    switch(t)
    {
    case LEFT_GRIPPER:
        prefix = "l_gripper_";
        break;

    case RIGHT_GRIPPER:
        prefix = "r_gripper_";
        break;

    case STANDARD_GRIPPER:
        prefix = prefixIn+"gripper_";
        break;

    default:
        prefix = "l_gripper_";
        break;
    }

    return prefix;
}

double JointStatePub::statePosJoint0(State s)
{
    double pos = 0.0;
    switch(s)
    {
    case OPEN_STATE:
        pos = -M_PI/180*30;
        break;

    case CLOSED_STATE:
        pos = 0.0;
        break;

    default:
        pos = -M_PI/180*30;
    }

    return pos;
}

double JointStatePub::statePosJoint1(State s)
{
    double pos = 0.0;
    switch(s)
    {
    case OPEN_STATE:
        pos = 0.0;
        break;

    case CLOSED_STATE:
        pos = M_PI/180*67;
        break;

    default:
        pos = 0.0;
    }

    return pos;
}

JointStatePub::JointStatePub(Type type,
        const QString& topicName, const QString &prefix) :
    m_type(type),
    m_topicName(topicName),
    m_prefix(prefix)
{
    ROS_INFO_STREAM("JointStatePub Gripper Type: "<<m_type);
    ROS_INFO_STREAM("JointStatePub Gripper Topic Name: "<<m_topicName.toStdString());
    ROS_INFO_STREAM("JointStatePub Gripper Prefix: "<<m_prefix.toStdString());
    if(m_topicName.isEmpty())
    {
        switch(m_type)
        {
        case LEFT_GRIPPER:
            m_topicName = "/left_fingers_joint_states";
            break;

        case RIGHT_GRIPPER:
            m_topicName = "/right_fingers_joint_states";
            break;
        //This case will not be needed since to use the prefix, we need to pass a topic name.
        //I added this option in case that the topic name used is "".
        case STANDARD_GRIPPER:
            m_topicName = "/"+m_prefix+"fingers_joint_states";
            break;
        default:
            m_topicName = "/left_fingers_joint_states";
        }
    }

    ROS_INFO_STREAM("Lacquey Gripper Topic Name: "<<m_topicName.toStdString());
    ROS_INFO_STREAM("Lacquey Gripper joint_state prefix: "<<m_prefix.toStdString());

    m_pub = m_node.advertise<sensor_msgs::JointState>(m_topicName.toLatin1().data(), 1000);

    setState(OPEN_STATE);
}

JointStatePub::~JointStatePub()
{

}

void JointStatePub::setState(State s)
{
    m_joints = fingerJoints(m_type,s,m_prefix);

//    for(int i=0; i<m_joints.size(); i++)
//    {
//        ROS_INFO_STREAM(m_joints.at(i).toString().toStdString());
//    }
}

void JointStatePub::publish()
{
    m_pub.publish(convToRosMsg(m_joints));
}

}}
