#pragma once

#include <Eigen/Dense>
#include "robotModel.hpp"

namespace robot::kinematics
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
Transform forwardKinematics(const model::Robot&);

// Compute the transforms of all joints/links in the robot
IKResult solveIK(model::Robot&, const Transform&, int iterations = 50, double lambda = 0.1);

// Compute joint angles to reach the desired end-effector position and set them in the robot model
bool inverseKinematics(model::Robot&, const Eigen::Vector3d&);

void exportForwardKinematics(const model::Robot&, const std::string&);
} // namespace robot
