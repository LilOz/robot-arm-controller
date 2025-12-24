#pragma once

#include "kinematics.hpp"
#include <chrono>
namespace robot
{

struct Waypoint
{
  std::chrono::milliseconds timestamp;
  Transform                 eeTransform;
};

struct Trajectory
{
  std::vector<Waypoint> waypoints;
};

Trajectory generateLinearTrajectory(const Transform& Ts, const Transform& Tt, double duration,
                                    int steps);

void exportTrajectory(const Trajectory& traj, const std::string& path);

} // namespace robot
