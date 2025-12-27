#include "robotModel.hpp"
#include "kinematics.hpp"
#include <iostream>

int main()
{
  using namespace robot::model;
  using namespace robot::kinematics;

  Robot robot;
  robot.loadFromJson("config/models/6dof_spherical_model.json");

  std::vector<Transform> poses;

  Transform T1;
  T1.p = {0.4, 0.1, 0.3};
  T1.R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  poses.push_back(T1);

  Transform T2;
  T2.p = T1.p;
  T2.R = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  poses.push_back(T2);

  for (size_t i = 0; i < poses.size(); ++i)
  {
    std::cout << "\n[6DOF IK] Target " << i+1 << "\n";

    auto result = solveIK6D(robot, poses[i], 500);
    if (result != IKResult::Success)
    {
      std::cout << "IK failed\n";
      continue;
    }

    auto T = forwardKinematics(robot);
    std::cout << "Reached RPY (deg): "
              << T.R.eulerAngles(0,1,2) * 180.0 / M_PI << "\n";
  }
}
