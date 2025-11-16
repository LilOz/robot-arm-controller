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

// Compute the end-effector position from the robot model
Transform forwardKinematics(const Robot& robot);
void      exportForwardKinematics(const Robot&, const std::string&);
} // namespace robot
