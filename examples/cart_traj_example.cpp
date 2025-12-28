#include "trajectory.hpp"
#include <iostream>

int main()
{
  using namespace robot::trajectory;
  using robot::kinematics::Transform;

  std::vector<Transform> targets;

  {
    // Flat circular path parameters
    const Eigen::Vector3d center(0.30, 0.00, 0.25); // circle center
    const double          radius = 0.15;
    const int             N = 10; // number of waypoints

    // End-effector always pointing up
    Eigen::Matrix3d R_up = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_down = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    // If your tool frame is not Z-up by default, adjust here:
    // R_up = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    for (int i = 0; i < N; ++i)
    {
      double theta = 2.0 * M_PI * static_cast<double>(i) / (N - 1);

      Transform T;
      // flat circle in XY plane
      T.p = center + Eigen::Vector3d(radius * std::cos(theta), radius * std::sin(theta), 0.0);
      if (i > N / 2)
        T.R = R_down;
      else
        T.R = R_up;

      targets.push_back(T);
    }
  }

  double duration = 10.0;
  int    steps = 500;

  Trajectory traj = generateCartesianSpline(targets, duration, steps);
  exportTrajectory(traj, "trajectory.csv");

  using namespace robot::model;
  using namespace robot::kinematics;

  Robot robot("config/models/6dof_spherical_model.json");
  solveIK(robot, traj.waypoints.front().eeTransform);

  TrajectoryJointSpace joint_traj = generateJointSpaceTrajectory(robot, traj);

  exportTrajectoryJointSpace(joint_traj, "trajectory_joint_space.csv");

  std::cout << "Generated " << traj.waypoints.size() << " trajectory waypoints\n";
}
