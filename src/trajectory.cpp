#include "trajectory.hpp"

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
    Eigen::Matrix3d R = q_interp.toRotationMatrix();

    Transform T;
    T.p = p;
    T.R = R;

    Waypoint wp;
    wp.timestamp = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(alpha * duration * 1000));
    wp.eeTransform = T;

    traj.waypoints.push_back(wp);
  }

  return traj;
}
} // namespace robot
