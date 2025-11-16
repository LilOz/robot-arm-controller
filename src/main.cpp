#include "kinematics.hpp"
#include "robotModel.hpp"
#include <iostream>

int main()
{

  robot::Robot myRobot;
  myRobot.loadFromJson("config/models/robot_model.json");
  robot::printRobot(myRobot);

  std::cout << "End-Effector Position: " << robot::forwardKinematics(myRobot).p.transpose() << "\n";
  std::cout << "End-Effector Rotation:\n"
            << robot::forwardKinematics(myRobot).R.eulerAngles(0, 1, 2) * 180 / M_PI << "\n";
  robot::exportForwardKinematics(myRobot, "forward_kinematics.csv");
  return 0;
}
