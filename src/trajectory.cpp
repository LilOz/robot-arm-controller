#include "trajectory.hpp"
#include <fstream>

namespace robot
{
Trajectory generateLinearTrajectory(const Transform& Ts, const Transform& Tt, double duration,
                                    int steps)
{
  Trajectory traj;
  traj.waypoints.reserve(steps + 1);

  for (int i = 0; i <= steps; ++i)
  {
    double alpha = static_cast<double>(i) / static_cast<double>(steps);

    // Interpolate position
    Eigen::Vector3d p = (1 - alpha) * Ts.p + alpha * Tt.p;

    // Interpolate rotation using Slerp
    Eigen::Quaterniond q_start(Ts.R);
    Eigen::Quaterniond q_target(Tt.R);
    Eigen::Quaterniond q_interp = q_start.slerp(alpha, q_target);
    Eigen::Matrix3d    R = q_interp.toRotationMatrix();

    Transform T;
    T.p = p;
    T.R = R;

    Waypoint wp;
    wp.timestamp = std::chrono::milliseconds(static_cast<int>(alpha * duration * 1000));
    wp.eeTransform = T;

    traj.waypoints.push_back(wp);
  }

  return traj;
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
    // (Optional) ensure a consistent sign to avoid visual discontinuities when plotting
    if (q.w() < 0.0)
      q.coeffs() *= -1.0;

    file << t_ms << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.w() << ',' << q.x()
         << ',' << q.y() << ',' << q.z() << '\n';
  }
}
} // namespace robot
