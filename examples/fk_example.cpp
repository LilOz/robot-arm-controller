#include "robotModel.hpp"
#include "kinematics.hpp"
#include <iostream>

int main()
{
  using namespace robot::model;
  using namespace robot::kinematics;

  Robot robot("config/models/6dof_spherical_model.json");
  printRobot(robot);

  auto T = forwardKinematics(robot);

  std::cout << "EE Position: " << T.p.transpose() << "\n";
  std::cout << "EE RPY (deg): "
            << T.R.eulerAngles(0,1,2) * 180.0 / M_PI << "\n";

  exportForwardKinematics(robot, "forward_kinematics.csv");
}
