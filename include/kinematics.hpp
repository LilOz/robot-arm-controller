#pragma once

#include <Eigen/Dense>
#include "robotModel.hpp"

namespace robot
{

// A simple rotation + translation transform
struct Transform
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

enum class IKResult
{
  Success,
  Unreachable,
  JointLimitBlocked,
  Diverged,
  MaxIterationsExceeded
};

// Compute the end-effector position from the robot model
Transform forwardKinematics(const Robot&);

// Compute the transforms of all joints/links in the robot
IKResult solveIK(Robot&, const Eigen::Vector3d&, int iterations = 50, double lambda = 0.1);
IKResult solveIK6D(Robot&, const Transform&, int iterations = 50, double lambda = 0.1);

// Compute joint angles to reach the desired end-effector position and set them in the robot model
bool inverseKinematics(Robot&, const Eigen::Vector3d&);

void exportForwardKinematics(const Robot&, const std::string&);
} // namespace robot
