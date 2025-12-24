#include "kinematics.hpp"
#include "trajectory.hpp"
#include "robotModel.hpp"
#include <iostream>

int main()
{
  robot::Robot myRobot;
  myRobot.loadFromJson("config/models/6dof_spherical_model.json");
  robot::printRobot(myRobot);

  std::cout << "Initial End-Effector Position: " << robot::forwardKinematics(myRobot).p.transpose()
            << "\n";
  std::cout << "Initial End-Effector Rotation:\n"
            << robot::forwardKinematics(myRobot).R.eulerAngles(0, 1, 2) * 180 / M_PI << "\n";

  // ----------------------------------------
  // 3-DOF POSITION-ONLY IK TESTS
  // ----------------------------------------
  std::vector<Eigen::Vector3d> targets = {
      Eigen::Vector3d(0.1, 0.2, 0.2),
      Eigen::Vector3d(0.3, -0.3, 0.4),
      Eigen::Vector3d(0.4, 0.4, 0.1),
      Eigen::Vector3d(0.6, 0.0, 0.3),
  };

  for (size_t i = 0; i < targets.size(); ++i)
  {
    std::cout << "\n[3DOF IK] Target " << i + 1 << ": " << targets[i].transpose() << "\n";
    auto result = robot::solveIK(myRobot, targets[i], 1000);

    if (result != robot::IKResult::Success)
    {
      std::cout << "IK failed with result code: " << static_cast<int>(result) << "\n";
      continue;
    }

    std::cout << "End-Effector Position: "
              << robot::forwardKinematics(myRobot).p.transpose() << "\n";
    std::cout << "End-Effector Rotation:\n"
              << robot::forwardKinematics(myRobot).R.eulerAngles(0, 1, 2) * 180 / M_PI << "\n";
  }

  // ----------------------------------------
  // 6-DOF POSITION + ORIENTATION IK TESTS
  // ----------------------------------------

  std::cout << "\n==============================\n";
  std::cout << "      6-DOF IK TESTS\n";
  std::cout << "==============================\n";

  // --- Create several orientation targets ---
  std::vector<robot::Transform> poseTargets;

  // Target 1: point forward + rotate 90° around Z
  {
    robot::Transform T;
    T.p = Eigen::Vector3d(0.4, 0.1, 0.3);
    T.R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    poseTargets.push_back(T);
  }

  // Target 2: same position, rotate −45° around X
  {
    robot::Transform T;
    T.p = Eigen::Vector3d(0.4, 0.1, 0.3);
    T.R = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
    poseTargets.push_back(T);
  }


  // Run tests
  for (size_t i = 0; i < poseTargets.size(); ++i)
  {
    std::cout << "\n[6DOF IK] Target Pose " << i + 1 << "\n";
    std::cout << "Target Position: " << poseTargets[i].p.transpose() << "\n";

    Eigen::Vector3d rpy = poseTargets[i].R.eulerAngles(0, 1, 2) * 180.0 / M_PI;
    std::cout << "Target Orientation RPY: " << rpy.transpose() << "\n";

    auto result = robot::solveIK6D(myRobot, poseTargets[i], 500);

    if (result != robot::IKResult::Success)
    {
      std::cout << "6-DOF IK failed with code: " << static_cast<int>(result) << "\n";
      continue;
    }

    auto Tfinal = robot::forwardKinematics(myRobot);
    Eigen::Vector3d rpyFinal = Tfinal.R.eulerAngles(0, 1, 2) * 180.0 / M_PI;

    std::cout << "FINAL Position: " << Tfinal.p.transpose() << "\n";
    std::cout << "FINAL Orientation RPY: " << rpyFinal.transpose() << "\n";
  }

  robot::exportForwardKinematics(myRobot, "forward_kinematics.csv");

  // Example start and target transforms
  robot::Transform Ts, Tt;
  Ts.p = Eigen::Vector3d(0, 0, 0);
  Ts.R = Eigen::Matrix3d::Identity();

  Tt.p = Eigen::Vector3d(1, 1, 1);
  Tt.R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  double duration = 2.0; // seconds
  int    steps = 10;

  robot::Trajectory traj = generateLinearTrajectory(Ts, Tt, duration, steps);

  // Print the generated waypoints
  for (size_t i = 0; i < traj.waypoints.size(); ++i)
  {
    const robot::Waypoint& wp = traj.waypoints[i];
    std::cout << "Waypoint " << i << ": position = [" << wp.eeTransform.p.transpose()
              << "], timestamp = " << wp.timestamp.count() << " ms" << std::endl;
  }
  robot::exportTrajectory(traj, "trajectory.csv");

  return 0;
}
