#pragma once

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>

namespace tum_ics_ur10_controller_tutorial
{

/**
 * @brief Force/Torque sensor interface
 *
 * Subscribes to F/T sensor topic and provides filtered force/torque data.
 * Used for admittance control and contact detection.
 */
class FTSensorInterface
{
public:
  FTSensorInterface();

  virtual ~FTSensorInterface() = default;

  /**
   * @brief Initialize sensor interface
   * @param nh ROS node handle
   * @return true if successful
   */
  bool init(ros::NodeHandle& nh);

  /**
   * @brief Get current force measurement
   * @return 3D force vector (N)
   */
  Eigen::Vector3d getForce() const;

  /**
   * @brief Get current torque measurement
   * @return 3D torque vector (Nm)
   */
  Eigen::Vector3d getTorque() const;

  /**
   * @brief Get complete wrench (force + torque)
   * @return 6D wrench vector [force; torque]
   */
  Eigen::Matrix<double, 6, 1> getWrench() const;

  /**
   * @brief Zero/calibrate sensor (bias removal)
   *
   * Records current reading as bias and subtracts it from future readings.
   */
  void zero();

  /**
   * @brief Check if sensor data is valid (recent)
   * @param max_age Maximum age in seconds
   * @return true if data is recent
   */
  bool isDataValid(double max_age = 0.1) const;

  /**
   * @brief Get magnitude of force vector
   * @return Force magnitude (N)
   */
  double getForceMagnitude() const;

private:
  // Sensor callback
  void sensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // Apply low-pass filter
  Eigen::Matrix<double, 6, 1> applyFilter(const Eigen::Matrix<double, 6, 1>& raw);

  // ROS
  ros::Subscriber sensor_sub_;

  // Current measurements
  Eigen::Vector3d force_;
  Eigen::Vector3d torque_;
  ros::Time last_update_;

  // Bias (for zeroing)
  Eigen::Vector3d force_bias_;
  Eigen::Vector3d torque_bias_;

  // Filter state (simple exponential moving average)
  Eigen::Matrix<double, 6, 1> filtered_wrench_;
  double filter_alpha_;  // Filter coefficient (0-1, higher = less filtering)

  // Configuration
  std::string sensor_topic_;
};

} // namespace tum_ics_ur10_controller_tutorial
