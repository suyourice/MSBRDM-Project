#ifndef LACQUEY_GRIPPER_DRIVER_JOINT_STATE_H
#define LACQUEY_GRIPPER_DRIVER_JOINT_STATE_H

#include "sensor_msgs/JointState.h"

#include <QVector>
#include <QString>

namespace tum_ics_lacquey_gripper_driver{

class JointState
{
public:
    static JointState Default();

private:
    QString m_name;
    double m_pos;

public:
    JointState(const QString& name="", double pos=0.0);
    JointState(const JointState& js);

    ~JointState();

    bool operator== (const JointState& other) const;
    bool operator!= (const JointState& other) const;

    const QString& name() const;
    double pos() const;

    QString toString() const;
};

sensor_msgs::JointState convToRosMsg(const QVector<JointState>& joints);

}

#endif // LACQUEY_GRIPPER_DRIVER_JOINT_STATE_H
