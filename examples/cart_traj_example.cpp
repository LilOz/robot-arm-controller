#include "trajectory.hpp"
#include <iostream>

int main()
{
  using namespace robot::trajectory;
  using robot::kinematics::Transform;

  std::vector<Transform> targets;

  // --- Target 0: reachable offset, no rotation ---
  Transform T0;
  T0.p = Eigen::Vector3d(0.35, 0.10, 0.15);
  T0.R = Eigen::Matrix3d::Identity();
  targets.push_back(T0);

  // --- Target 1: rotate about Z (yaw), moderate translation ---
  Transform T1;
  T1.p = Eigen::Vector3d(0.30, -0.20, 0.25);
  T1.R = Eigen::AngleAxisd(M_PI / 4.0, // 45 deg
                           Eigen::Vector3d::UnitZ())
             .toRotationMatrix();
  targets.push_back(T1);

  // --- Target 2: rotate about Y (pitch), higher Z ---
  Transform T2;
  T2.p = Eigen::Vector3d(0.20, 0.30, 0.35);
  T2.R = Eigen::AngleAxisd(-M_PI / 3.0, // -60 deg
                           Eigen::Vector3d::UnitY())
             .toRotationMatrix();
  targets.push_back(T2);

  // --- Target 3: rotate about X (roll), stretched reach ---
  Transform T3;
  T3.p = Eigen::Vector3d(0.45, 0.00, 0.20);
  T3.R = Eigen::AngleAxisd(M_PI / 2.0, // 90 deg
                           Eigen::Vector3d::UnitX())
             .toRotationMatrix();
  targets.push_back(T3);

  // --- Target 4: rotate about Z again, near workspace boundary ---
  Transform T4;
  T4.p = Eigen::Vector3d(0.15, -0.45, 0.30);
  T4.R = Eigen::AngleAxisd(-3.0 * M_PI / 4.0, // -135 deg
                           Eigen::Vector3d::UnitZ())
             .toRotationMatrix();
  targets.push_back(T4);

  // --- Target 5: rotate about Y, near-singular wrist posture ---
  Transform T5;
  T5.p = Eigen::Vector3d(0.25, 0.00, 0.45);
  T5.R = Eigen::AngleAxisd(M_PI / 2.0, // 90 deg
                           Eigen::Vector3d::UnitY())
             .toRotationMatrix();
  targets.push_back(T5);

  double duration = 5.0;
  int    steps = 200;

  Trajectory traj = generateCartesianSpline(targets, duration, steps);
  exportTrajectory(traj, "trajectory.csv");

  using namespace robot::model;
  Robot robot;
  robot.loadFromJson("config/models/6dof_spherical_model.json");
  TrajectoryJointSpace joint_traj = generateJointSpaceTrajectory(robot, traj);

  exportTrajectoryJointSpace(joint_traj, "trajectory_joint_space.csv");

  std::cout << "Generated " << traj.waypoints.size() << " trajectory waypoints\n";
}
