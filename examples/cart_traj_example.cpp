#include "trajectory.hpp"
#include <iostream>

int main()
{
  using namespace robot::trajectory;
  using robot::kinematics::Transform;

  std::vector<Transform> targets;

  // Start: origin, identity
  Transform T0;
  T0.p = Eigen::Vector3d(0.0, 0.0, 0.0);
  T0.R = Eigen::Matrix3d::Identity(); // Move forward + up, yaw right 45°
  Transform T1;
  T1.p = Eigen::Vector3d(0.3, 0.0, 0.2);
  T1.R = Eigen::AngleAxisd(M_PI / 4.0,
                           Eigen::Vector3d::UnitY())
             .toRotationMatrix(); // Move diagonally, pitch up 90°
  Transform T2;
  T2.p = Eigen::Vector3d(0.6, 0.2, 0.4);
  T2.R = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix(); //
  // Move up + forward, yaw 180°
  Transform T3;
  T3.p = Eigen::Vector3d(0.8, 0.0, 0.6);
  T3.R = Eigen::AngleAxisd(M_PI,
                           Eigen::Vector3d::UnitY())
             .toRotationMatrix(); // Pull back in Y, roll 90°
  Transform T4;
  T4.p = Eigen::Vector3d(0.7, -0.2, 0.6);
  T4.R = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(); //
  // Sweep left, combined yaw + pitch
  Transform T5;
  T5.p = Eigen::Vector3d(0.4, -0.3, 0.4);
  T5.R = (Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitX()))
             .toRotationMatrix();
  // // Come back toward origin, near-singular orientation
  Transform T6;
  T6.p = Eigen::Vector3d(0.2, -0.1, 0.2);
  T6.R = (Eigen::AngleAxisd(M_PI * 0.9, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI * 0.9,
                            Eigen::Vector3d::UnitY()))
             .toRotationMatrix(); // Final pose: slight offset,
                                  // identity again (tests full loop-back)
  Transform T7;
  T7.p = Eigen::Vector3d(0.1, 0.0, 0.1);
  T7.R = Eigen::Matrix3d::Identity(); // Push all targets
  targets.push_back(T0);
  targets.push_back(T1);
  targets.push_back(T2);
  targets.push_back(T3);
  targets.push_back(T4);
  targets.push_back(T5);
  targets.push_back(T6);
  targets.push_back(T7);

  double duration = 5.0;
  int    steps = 200;

  Trajectory traj = generateCartesianSpline(targets, duration, steps);
  exportTrajectory(traj, "trajectory.csv");

  std::cout << "Generated " << traj.waypoints.size() << " trajectory waypoints\n";
}
