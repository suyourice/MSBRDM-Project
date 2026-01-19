#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class SimpleEffortControl : public ControlEffort
    {
    private:
      bool is_first_iter_;

      Vector6d q_start_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;

      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;

      Matrix6d Kp_;
      Matrix6d Kd_;
      Matrix6d Ki_;

      Vector6d q_goal_;
      double spline_period_;

      Vector6d delta_q_;
      Vector6d delta_qp_;

    public:
      SimpleEffortControl(double weight = 1.0, const QString &name = "SimpleEffortCtrl");

      ~SimpleEffortControl();

      void setQInit(const JointState &q_init);

      void setQHome(const JointState &q_home);

      void setQPark(const JointState &q_park);

    private:
      bool init();

      bool start();

      Vector6d update(const RobotTime &time, const JointState &state);

      bool stop();
    };

  } // namespace RobotControllers
} // namespace tuics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
