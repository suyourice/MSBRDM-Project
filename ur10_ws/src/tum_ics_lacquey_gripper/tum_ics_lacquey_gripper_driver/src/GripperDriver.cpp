#include <tum_ics_lacquey_gripper_driver/GripperDriver.h>

#include <tum_ics_lacquey_gripper_msgs/GripperState.h>

#include <unistd.h>
#include <math.h>

#include <QElapsedTimer>
#include <QHostAddress>
#include <QDataStream>

namespace tum_ics_lacquey_gripper_driver{
namespace Gripper{

QString GripperDriver::defaultRobotIpAddr(Type t)
{
    switch (t)
    {
    case RIGHT_GRIPPER:
        return "192.168.1.5";

    case LEFT_GRIPPER:
        return "192.168.1.6";

    default:
        return "192.168.1.5";
    }
}

QVector<QString> GripperDriver::commands()
{
    const QVector<QString> cmds = QVector<QString>()
            << "open"
            << "close"
            << "free";

    return cmds;
}


GripperDriver::GripperDriver(Type type,
                             const QString &pcIpAddr,
                             const QString &robotIpAddr,
                             quint16 gripperPort, const bool realGripper, const QString &prefixIn,
                             const QString& jointStateTopicName) :
    m_type(type),
    m_pcIp(pcIpAddr),
    m_robotIp(robotIpAddr),
    m_gripperPort(gripperPort),
    m_server(0),
    m_socket(0),
    m_started(false),
    m_state("free"),
    m_lastState("free"),
    m_pendingState("free"),
    m_jointStatePub(type,jointStateTopicName,prefixIn),
    m_prefix(prefixIn),
    m_realGripper(realGripper)
{
    if(m_robotIp.isEmpty())
    {
        m_robotIp = defaultRobotIpAddr(m_type);
    }

    QString name;

    if(m_type!=STANDARD_GRIPPER)
    {
        QString param;
        if(m_type == RIGHT_GRIPPER)
        {
            param = "~script_manager_prefix/right";
        }
        else if (m_type == LEFT_GRIPPER)
        {
            param = "~script_manager_prefix/left";
        }

        QString prefix;
        if(hasParam(param))
        {
            getParamString(prefix,param);
            ROS_INFO("script manager prefix: '%s'",prefix.toLatin1().data());
        }


        name = prefix + "setScriptManagerState";
    }
    else
    {
        name = "setScriptManagerState";
    }


    if(m_realGripper)
    {
        ROS_INFO_STREAM("service: "<<name.toStdString());

        m_setScriptState = m_node.serviceClient
                <tum_ics_ur_robot_msgs::setScriptManagerState>
                (name.toStdString());
    }

    m_setState = m_node.advertiseService(
                "setGripperState",
                &GripperDriver::setStateCallback,
                this);

    m_getState = m_node.advertiseService(
                "getGripperState",
                &GripperDriver::getStateCallback,
                this);

    m_statePub = m_node.advertise<tum_ics_lacquey_gripper_msgs::GripperState>
            ("gripperState", 100);
}

GripperDriver::~GripperDriver()
{
    stop();
}

bool GripperDriver::start()
{
    if(m_started)
    {
        return false;
    }

    if(m_realGripper)
    {
        // make sure that the program in the script is NOT running
        if(!enableScript(false))
        {
            return false;
        }

        usleep(2000*1000);

        // start the server
        m_server = new QTcpServer();
        if(!m_server->listen(QHostAddress(m_pcIp), m_gripperPort))
        {
            ROS_ERROR("GripperDriver: Server listen failed with error '%s'",
                      m_server->errorString().toLatin1().data());
            return false;
        }

        if(!enableScript(true))
        {
            if(m_server->isListening())
            {
                m_server->close();
            }
            delete m_server;
            m_server = 0;
            return false;
        }

        // the connection should now already be on the server
        if(!m_server->waitForNewConnection(100))
        {
            ROS_ERROR("GripperDriver: Server has no client connection.");

            if(!enableScript(false))
            {
                ROS_ERROR("GripperDriver: Stopping script thread failed");
            }

            if(m_server->isListening())
            {
                m_server->close();
            }
            delete m_server;
            m_server = 0;
            return false;
        }

        m_socket = m_server->nextPendingConnection();
        if(m_socket == 0)
        {
            ROS_ERROR("GripperDriver: Getting client socket failed");

            if(!enableScript(false))
            {
                ROS_ERROR("GripperDriver: Stopping script thread failed");
            }

            if(m_server->isListening())
            {
                m_server->close();
            }
            delete m_server;
            m_server = 0;
            return false;
        }
    }

    m_started = true;
    return true;
}

void GripperDriver::stop()
{
    if(!m_started)
    {
        return;
    }

    if(m_realGripper)
    {
        if(!enableScript(false))
        {
            ROS_ERROR("GripperDriver: Stopping script thread failed");
        }

        m_socket->close();

        if(m_server->isListening())
        {
            m_server->close();
        }

        delete m_server;
    }
    m_started = false;
    m_server = 0;
}

bool GripperDriver::send(const QString& cmd)
{
    if(!m_started)
    {
        ROS_ERROR("Error: Gripper not started!");
        return false;
    }

    if(!commands().contains(cmd))
    {
        ROS_ERROR("Error: Unknown command '%s'",cmd.toLatin1().data());
        return false;
    }

    if(m_state == "busy")
    {
        return false;
    }

    if(cmd == "open")
    {
        if(m_realGripper)
        {
            if(!send(ACTIVATE_GRIPPER_CMD))
            {
                return false;
            }

            if(!send(OPEN_GRIPPER_CMD))
            {
                return false;
            }
        }
        m_jointStatePub.setState(OPEN_STATE);

        m_timer.start();
        m_lastState = m_state;
        m_pendingState = "open";
        m_state = "busy";
        //        m_state = "close";
        return true;
    }
    else if(cmd == "close")
    {
        if(m_realGripper)
        {
            if(!send(ACTIVATE_GRIPPER_CMD))
            {
                return false;
            }

            if(!send(CLOSE_GRIPPER_CMD))
            {
                return false;
            }
        }
        m_jointStatePub.setState(CLOSED_STATE);
        m_timer.start();
        m_lastState = m_state;
        m_pendingState = "close";
        m_state = "busy";
        return true;
    }
    else if(cmd == "free")
    {
        if(m_realGripper)
        {
            if(!send(DEACTIVATE_GRIPPER_CMD))
            {
                return false;
            }
        }
        m_timer.start();
        m_lastState = m_state;
        m_pendingState = "free";
        m_state = "busy";
        return true;
    }

    ROS_ERROR("Error: Unknown command '%s'",cmd.toLatin1().data());
    return false;
}

void GripperDriver::update()
{
    if(m_timer.elapsed() > 1000)
    {
        m_state = m_pendingState;
        m_lastState = m_pendingState;
    }
}

void GripperDriver::publish()
{
    m_jointStatePub.publish();

    tum_ics_lacquey_gripper_msgs::GripperState msg;
    if(m_state == "open")
    {
        msg.stateId = 0;
    }
    else if(m_state == "close")
    {
        msg.stateId = 1;
    }
    else
    {
        msg.stateId = 2;
    }

    //    msg.state = m_state.toStdString();

    msg.state = m_lastState.toStdString();
    m_statePub.publish(msg);
}

bool GripperDriver::enableScript(bool enable)
{
    tum_ics_ur_robot_msgs::setScriptManagerState srv;
    srv.request.name="gripper";
    srv.request.enable = enable;

    if(!m_setScriptState.call(srv))
    {
        ROS_ERROR("GripperDriver: Failed to call service 'setScriptManagerState'");
        return false;
    }

    if(!srv.response.ok)
    {
        ROS_ERROR("GripperDriver: Calling service 'setScriptManagerState' failed.");
        return false;
    }

    return true;
}

bool GripperDriver::send(Command cmd)
{
    QByteArray data;
    QDataStream s(&data, QIODevice::ReadWrite);
    s << qint32(cmd);

    // clear rx buffer
    m_socket->waitForReadyRead(0);
    m_socket->readAll();

    m_socket->write(data);
    if(!m_socket->waitForBytesWritten())
    {
        ROS_INFO("GripperDriver: Write to the socket failed.");
        return false;
    }

    return true;
}

bool GripperDriver::setStateCallback(
        tum_ics_lacquey_gripper_msgs::setGripperState::Request &req,
        tum_ics_lacquey_gripper_msgs::setGripperState::Response &res)
{
    QString newState = QString(req.newState.c_str());

    if(!send(newState))
    {
        ROS_ERROR("setGripperState: set %s failed.",
                  newState.toLatin1().data());
        res.ok = false;
        return true;
    }

    res.ok = true;
    return true;
}

bool GripperDriver::getStateCallback(
        tum_ics_lacquey_gripper_msgs::getGripperState::Request &req,
        tum_ics_lacquey_gripper_msgs::getGripperState::Response &res)
{
    int size = commands().size();
    res.states.resize(size);

    QVector<QString> cmds = commands();

    for(int i=0; i<size; i++)
    {
        res.states[i] = cmds.at(i).toStdString();
        res.currentState = m_state.toStdString();
    }

    return true;
}

bool GripperDriver::getParamString(QString& str, const QString& param)
{
    str.clear();

    std::string p = param.toStdString();
    std::string stdStr;

    if(!ros::param::has(p))
    {
        ROS_ERROR("Parameter '%s' is missing.", p.c_str());
        return false;
    }

    if(!ros::param::get(p,stdStr))
    {
        ROS_ERROR("Couldn't get parameter '%s'.", p.c_str());
        return false;
    }

    str = QString(stdStr.c_str());
    return true;
}

bool GripperDriver::hasParam(const QString& param)
{
    std::string p = param.toStdString();
    return ros::param::has(p);
}

}}
