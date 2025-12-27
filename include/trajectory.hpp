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

Trajectory generateLinearTrajectory(const kinematics::Transform& Ts,
                                    const kinematics::Transform& Tt, double duration, int steps,
                                    double accTime);

void exportTrajectory(const Trajectory& traj, const std::string& path);

} // namespace robot::trajectory
