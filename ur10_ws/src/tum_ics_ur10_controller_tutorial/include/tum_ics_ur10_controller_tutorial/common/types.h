#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tum_ics_ur10_controller_tutorial
{

// Common type aliases for readability
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Joint state structure
struct JointState
{
  Eigen::VectorXd q;    // Joint positions
  Eigen::VectorXd qp;   // Joint velocities
  Eigen::VectorXd qpp;  // Joint accelerations
  Eigen::VectorXd tau;  // Joint torques

  JointState() = default;

  explicit JointState(size_t dof)
    : q(Eigen::VectorXd::Zero(dof)),
      qp(Eigen::VectorXd::Zero(dof)),
      qpp(Eigen::VectorXd::Zero(dof)),
      tau(Eigen::VectorXd::Zero(dof))
  {}
};

// Cartesian pose structure
struct CartesianPose
{
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;

  CartesianPose()
    : position(Eigen::Vector3d::Zero()),
      rotation(Eigen::Matrix3d::Identity())
  {}

  explicit CartesianPose(const Eigen::Affine3d& pose)
    : position(pose.translation()),
      rotation(pose.rotation())
  {}

  Eigen::Affine3d toAffine() const
  {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = position;
    pose.linear() = rotation;
    return pose;
  }
};

// Wrench structure (force/torque)
struct Wrench
{
  Eigen::Vector3d force;
  Eigen::Vector3d torque;

  Wrench()
    : force(Eigen::Vector3d::Zero()),
      torque(Eigen::Vector3d::Zero())
  {}
};

} // namespace tum_ics_ur10_controller_tutorial
