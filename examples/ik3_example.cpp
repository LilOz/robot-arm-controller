#include "robotModel.hpp"
#include "kinematics.hpp"
#include <iostream>

int main()
{
  using namespace robot::model;
  using namespace robot::kinematics;

  Robot robot;
  robot.loadFromJson("config/models/6dof_spherical_model.json");

  std::vector<Eigen::Vector3d> targets = {
    {0.1,  0.2, 0.2},
    {0.3, -0.3, 0.4},
    {0.4,  0.4, 0.1},
    {0.6,  0.0, 0.3}
  };

  for (size_t i = 0; i < targets.size(); ++i)
  {
    std::cout << "\n[3DOF IK] Target " << i+1 << ": "
              << targets[i].transpose() << "\n";

    auto result = solveIK(robot, targets[i], 1000);
    if (result != IKResult::Success)
    {
      std::cout << "IK failed\n";
      continue;
    }

    auto T = forwardKinematics(robot);
    std::cout << "Reached: " << T.p.transpose() << "\n";
  }
}
