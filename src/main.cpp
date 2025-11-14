#include "kinematics.hpp"
#include "robotModel.hpp"
#include <iostream>
int main()
{

  robot::Robot myRobot;
  myRobot.loadFromJson("config/models/robot_model.json");
  robot::printRobot(myRobot);

  std::cout << "End-Effector Position: " << robot::forwardKinematics(myRobot).transpose() << "\n";
  return 0;
}
