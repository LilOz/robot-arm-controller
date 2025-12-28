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

struct WaypointJointSpace
{
  std::chrono::milliseconds timestamp;
  std::vector<double>       jointAngles;
};

struct Trajectory
{
  std::vector<Waypoint> waypoints;
};

struct TrajectoryJointSpace
{
  std::vector<WaypointJointSpace> jointAngles;
};

Trajectory generateCartesianSpline(const std::vector<kinematics::Transform>& targets,
                                   double duration, int steps);

TrajectoryJointSpace generateJointSpaceTrajectory(model::Robot& robot, const Trajectory& cartesian_traj);

void exportTrajectory(const Trajectory& traj, const std::string& path);
void exportTrajectoryJointSpace(const TrajectoryJointSpace& traj, const std::string& path);

} // namespace robot::trajectory
