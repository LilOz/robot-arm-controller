#include "trajectory.hpp"
#include <fstream>
#include <iostream>

namespace robot::trajectory
{
Trajectory generateCartesianSpline(const std::vector<kinematics::Transform>& targets,
                                   double duration, int steps)
{
  assert(targets.size() >= 2);

  Trajectory traj;
  traj.waypoints.reserve(steps + 1);

  const int    N = static_cast<int>(targets.size());
  const double dt = duration / steps;

  std::vector<Eigen::Vector3d>    P(N);
  std::vector<Eigen::Quaterniond> Q(N);

  for (int i = 0; i < N; ++i)
  {
    P[i] = targets[i].p;
    Q[i] = Eigen::Quaterniond(targets[i].R).normalized();
  }

  // Enforce hemisphere consistency globally
  for (int i = 1; i < N; ++i)
  {
    if (Q[i - 1].dot(Q[i]) < 0.0)
      Q[i].coeffs() *= -1.0;
  }

  const int segmentCount = N - 1;
  const int baseSteps = steps / segmentCount;
  const int remainder = steps % segmentCount;

  int stepIdx = 0;

  for (int s = 0; s < segmentCount; ++s)
  {
    const int stepsThisSegment = baseSteps + (s == segmentCount - 1 ? remainder : 0);

    const auto& p1 = P[s];
    const auto& p2 = P[s + 1];

    const auto p0 = (s == 0) ? p1 - (p2 - p1) : P[s - 1];

    const auto p3 = (s + 2 < N) ? P[s + 2] : p2 + (p2 - p1);

    for (int i = 0; i <= stepsThisSegment; ++i)
    {
      if (s > 0 && i == 0)
        continue;

      double u = static_cast<double>(i) / stepsThisSegment;

      // --- Position: Catmull–Rom ---
      Eigen::Vector3d pos =
          0.5 * ((2.0 * p1) + (-p0 + p2) * u + (2 * p0 - 5 * p1 + 4 * p2 - p3) * u * u +
                 (-p0 + 3 * p1 - 3 * p2 + p3) * u * u * u);

      // --- Orientation: SLERP ---
      Eigen::Quaterniond rot = Q[s].slerp(u, Q[s + 1]);

      kinematics::Transform T;
      T.p = pos;
      T.R = rot.toRotationMatrix();

      Waypoint wp;
      wp.timestamp = std::chrono::milliseconds(static_cast<int>(stepIdx * dt * 1000.0));
      wp.eeTransform = T;

      traj.waypoints.push_back(wp);
      ++stepIdx;
    }
  }

  return traj;
}

TrajectoryJointSpace generateJointSpaceTrajectory(model::Robot&     robot,
                                                  const Trajectory& cartesian_traj)
{
  using namespace robot::kinematics;

  TrajectoryJointSpace joint_traj;

  joint_traj.jointAngles.reserve(cartesian_traj.waypoints.size());

  for (const auto& wp : cartesian_traj.waypoints)
  {
    auto result = solveIK(robot, wp.eeTransform, 500);
    if (result != IKResult::Success)
    {
      std::cerr << "generateJointSpaceTrajectory: IK failed for waypoint at t = "
                << wp.timestamp.count() << " ms\n"
                << "Error code: " << static_cast<int>(result) << "\n";

      std::cerr << "Target position: " << wp.eeTransform.p.transpose() << "\n";

      std::cerr << "Target RPY (deg): "
                << (wp.eeTransform.R.eulerAngles(0, 1, 2) * 180.0 / M_PI).transpose() << "\n";
      throw std::runtime_error("generateJointSpaceTrajectory: IK failed for a waypoint");
    }
    std::vector<double> angles;
    for (const auto& link : robot.links)
    {
      angles.push_back(link.angle);
    }
    joint_traj.jointAngles.push_back({wp.timestamp, angles});
  }

  return joint_traj;
}

void exportTrajectory(const Trajectory& traj, const std::string& path)
{
  if (traj.waypoints.empty())
    throw std::runtime_error("exportTrajectoryToCsv: trajectory has no waypoints");

  std::ofstream file(path);
  if (!file.is_open())
    throw std::runtime_error("exportTrajectoryToCsv: failed to open file: " + path);

  file << "t_ms,x,y,z,qw,qx,qy,qz\n";
  file << std::fixed << std::setprecision(9);

  const auto t0 = traj.waypoints.front().timestamp;

  for (const auto& wp : traj.waypoints)
  {
    const auto t_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(wp.timestamp - t0).count();

    const Eigen::Vector3d& p = wp.eeTransform.p;

    Eigen::Quaterniond q(wp.eeTransform.R);

    file << t_ms << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.w() << ',' << q.x()
         << ',' << q.y() << ',' << q.z() << '\n';
  }
}

void exportTrajectoryJointSpace(const TrajectoryJointSpace& traj, const std::string& path)
{
  if (traj.jointAngles.empty())
    throw std::runtime_error("exportTrajectoryJointSpaceToCsv: trajectory has no waypoints");

  std::ofstream file(path);
  if (!file.is_open())
    throw std::runtime_error("exportTrajectoryJointSpaceToCsv: failed to open file: " + path);

  // Header
  file << "t_ms";
  for (size_t j = 0; j < traj.jointAngles.front().jointAngles.size(); ++j)
  {
    file << ",joint_" << (j + 1);
  }
  file << '\n';

  file << std::fixed << std::setprecision(9);

  const auto t0 = traj.jointAngles.front().timestamp;

  for (const auto& wp : traj.jointAngles)
  {
    const auto t_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(wp.timestamp - t0).count();

    file << t_ms;
    for (const auto& angle : wp.jointAngles)
    {
      file << ',' << angle;
    }
    file << '\n';
  }
}
} // namespace robot::trajectory
