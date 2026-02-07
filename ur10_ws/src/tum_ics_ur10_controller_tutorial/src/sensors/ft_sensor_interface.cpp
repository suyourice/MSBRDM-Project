#include <tum_ics_ur10_controller_tutorial/sensors/ft_sensor_interface.h>

namespace tum_ics_ur10_controller_tutorial
{

FTSensorInterface::FTSensorInterface()
  : filter_alpha_(0.2),
    sensor_topic_("/ft_sensor/wrench")
{
  force_.setZero();
  torque_.setZero();
  force_bias_.setZero();
  torque_bias_.setZero();
  filtered_wrench_.setZero();
}

bool FTSensorInterface::init(ros::NodeHandle& nh)
{
  ROS_INFO("FTSensorInterface::init()");

  // Read parameters
  nh.param("ft_sensor/topic", sensor_topic_, sensor_topic_);
  nh.param("ft_sensor/filter_alpha", filter_alpha_, 0.2);

  // Clamp filter alpha to valid range
  filter_alpha_ = std::max(0.0, std::min(1.0, filter_alpha_));

  // Subscribe to sensor topic
  sensor_sub_ = nh.subscribe(sensor_topic_, 1,
                             &FTSensorInterface::sensorCallback, this);

  ROS_INFO_STREAM("F/T Sensor interface initialized:");
  ROS_INFO_STREAM("  Topic: " << sensor_topic_);
  ROS_INFO_STREAM("  Filter alpha: " << filter_alpha_);

  return true;
}

Eigen::Vector3d FTSensorInterface::getForce() const
{
  return force_ - force_bias_;
}

Eigen::Vector3d FTSensorInterface::getTorque() const
{
  return torque_ - torque_bias_;
}

Eigen::Matrix<double, 6, 1> FTSensorInterface::getWrench() const
{
  Eigen::Matrix<double, 6, 1> wrench;
  wrench.head<3>() = getForce();
  wrench.tail<3>() = getTorque();
  return wrench;
}

void FTSensorInterface::zero()
{
  force_bias_ = force_;
  torque_bias_ = torque_;

  ROS_INFO("F/T sensor zeroed (bias removed)");
  ROS_INFO_STREAM("  Force bias: " << force_bias_.transpose());
  ROS_INFO_STREAM("  Torque bias: " << torque_bias_.transpose());
}

bool FTSensorInterface::isDataValid(double max_age) const
{
  if (last_update_.isZero())
    return false;

  double age = (ros::Time::now() - last_update_).toSec();
  return age < max_age;
}

double FTSensorInterface::getForceMagnitude() const
{
  return getForce().norm();
}

void FTSensorInterface::sensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  // Extract raw wrench
  Eigen::Matrix<double, 6, 1> raw_wrench;
  raw_wrench(0) = msg->wrench.force.x;
  raw_wrench(1) = msg->wrench.force.y;
  raw_wrench(2) = msg->wrench.force.z;
  raw_wrench(3) = msg->wrench.torque.x;
  raw_wrench(4) = msg->wrench.torque.y;
  raw_wrench(5) = msg->wrench.torque.z;

  // Apply filter
  filtered_wrench_ = applyFilter(raw_wrench);

  // Update measurements
  force_(0) = filtered_wrench_(0);
  force_(1) = filtered_wrench_(1);
  force_(2) = filtered_wrench_(2);
  torque_(0) = filtered_wrench_(3);
  torque_(1) = filtered_wrench_(4);
  torque_(2) = filtered_wrench_(5);

  last_update_ = msg->header.stamp;
}

Eigen::Matrix<double, 6, 1> FTSensorInterface::applyFilter(
  const Eigen::Matrix<double, 6, 1>& raw)
{
  // Simple exponential moving average (low-pass filter)
  // filtered = alpha * raw + (1 - alpha) * filtered_prev
  //
  // alpha = 1.0: no filtering (use raw)
  // alpha = 0.0: infinite filtering (no update)
  // alpha = 0.2: moderate filtering (recommended)

  if (filtered_wrench_.isZero())
  {
    // First sample, initialize filter
    return raw;
  }

  return filter_alpha_ * raw + (1.0 - filter_alpha_) * filtered_wrench_;
}

} // namespace tum_ics_ur10_controller_tutorial
