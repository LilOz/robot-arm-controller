#pragma once

#include "kinematics.hpp"
#include <chrono>
namespace robot::trajectory
{

struct Waypoint
{
  std::chrono::milliseconds timestamp;
  kinematics::Transform     eeTransform;
};

struct Trajectory
{
  std::vector<Waypoint> waypoints;
};

Trajectory generateCartesianSpline(const std::vector<kinematics::Transform>& targets,
                                   double duration, int steps);

void exportTrajectory(const Trajectory& traj, const std::string& path);

} // namespace robot::trajectory
