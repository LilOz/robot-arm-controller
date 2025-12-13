#pragma once

#include "kinematics.hpp"
#include <chrono>
namespace robot
{

struct Waypoint
{
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
  Transform eeTransform;
};

struct Trajectory
{
  std::vector<Waypoint> waypoints;
};

Trajectory generateLinearTrajectory(const Transform& Ts, const Transform& Tt, double duration,
                                    int steps);

} // namespace robot
