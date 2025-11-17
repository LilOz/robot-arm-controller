#include "kinematics.hpp"
#include "robotModel.hpp"
#include <iostream>

int main()
{

  robot::Robot myRobot;
  // myRobot.loadFromJson("config/models/robot_model.json");
  myRobot.loadFromJson("config/models/6dof_spherical_model.json");
  robot::printRobot(myRobot);

  std::cout << "End-Effector Position: " << robot::forwardKinematics(myRobot).p.transpose() << "\n";
  std::cout << "End-Effector Rotation:\n"
            << robot::forwardKinematics(myRobot).R.eulerAngles(0, 1, 2) * 180 / M_PI << "\n";

  std::vector<Eigen::Vector3d> targets = {
      Eigen::Vector3d(0.1, 0.2, 0.2),
      Eigen::Vector3d(0.3, -0.3, 0.4),
      Eigen::Vector3d(0.4, 0.4, 0.1),
      Eigen::Vector3d(0.6, 0.0, 0.3),
      // Eigen::Vector3d(2.6, 0.0, 0.3), // unreachable
  };
  for (size_t i = 0; i < targets.size(); ++i)
  {
    std::cout << "\nTarget " << i + 1 << ": " << targets[i].transpose() << "\n";
    auto result = robot::solveIK(myRobot, targets[i], 1000);
    if (result != robot::IKResult::Success)
    {
      std::cout << "IK failed with result code: " << static_cast<int>(result) << "\n";
      continue;
    }
    std::cout << "End-Effector Position: " << robot::forwardKinematics(myRobot).p.transpose()
              << "\n";
    std::cout << "End-Effector Rotation:\n"
              << robot::forwardKinematics(myRobot).R.eulerAngles(0, 1, 2) * 180 / M_PI << "\n";
  }

  robot::exportForwardKinematics(myRobot, "forward_kinematics.csv");

  return 0;
}
