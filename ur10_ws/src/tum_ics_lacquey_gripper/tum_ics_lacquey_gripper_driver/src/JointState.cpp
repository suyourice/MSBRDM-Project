#include <tum_ics_lacquey_gripper_driver/JointState.h>

namespace tum_ics_lacquey_gripper_driver{


JointState JointState::Default()
{
    return JointState();
}


JointState::JointState(const JointState& js)
{
    m_name  = js.m_name;
    m_pos   = js.m_pos;
}

JointState::JointState(const QString& name, double pos)
{
    m_name  = name;
    m_pos   = pos;
}

JointState::~JointState()
{

}

bool JointState::operator== (const JointState& other) const
{
    if(m_name != other.m_name)
    {
        return false;
    }

    if(m_pos != other.m_pos)
    {
        return false;
    }

    return true;
}

bool JointState::operator!= (const JointState& other) const
{
    return !(*this == other);
}

const QString& JointState::name() const
{
    return m_name;
}

double JointState::pos() const
{
    return m_pos;
}

QString JointState::toString() const
{
    QString s = "";

    s.append(QString().sprintf("jointState: %s, pos: %f",
                               m_name.toLatin1().data(), m_pos));

    return s;
}

sensor_msgs::JointState convToRosMsg(const QVector<JointState>& joints)
{
    sensor_msgs::JointState js;

    int size = joints.size();

    js.name.resize(size);
    js.position.resize(size);

    for(int i=0; i<size; i++)
    {
        js.name[i]         = joints.at(i).name().toStdString();
        js.position[i]     = joints.at(i).pos();
    }

    return js;
}

}
