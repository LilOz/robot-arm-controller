#include "robotModel.hpp"
#include "kinematics.hpp"
#include <iostream>

int main()
{
  using namespace robot::model;
  using namespace robot::kinematics;

  Robot robot("config/models/6dof_spherical_model.json");

  std::vector<Transform> poses;

  // 1) Nominal pose
  {
    Transform T;
    T.p = {0.4, 0.1, 0.3};
    T.R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    poses.push_back(T);
  }

  // 2) Same position, different roll
  {
    Transform T;
    T.p = {0.4, 0.1, 0.3};
    T.R = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
    poses.push_back(T);
  }

  // 3) Different position, mixed orientation
  {
    Transform T;
    T.p = {0.3, -0.25, 0.45};
    T.R = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ());
    poses.push_back(T);
  }

  // 4) Wrist near singularity (Z aligned)
  {
    Transform T;
    T.p = {0.25, 0.0, 0.5};
    T.R = Eigen::Matrix3d::Identity();
    poses.push_back(T);
  }

  // 5) Near reach boundary
  {
    Transform T;
    T.p = {0.55, 0.0, 0.15}; // adjust if this is unreachable
    T.R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    poses.push_back(T);
  }

  // 6) faceing down
  {
    Transform T;
    T.p = {0.3, 0.2, 0.4};
    T.R = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    poses.push_back(T);
  }

  for (size_t i = 0; i < poses.size(); ++i)
  {
    const auto& target = poses[i];

    std::cout << "\n[6DOF IK] Target " << i + 1 << "\n";

    std::cout << "Target position: " << target.p.transpose() << "\n";

    std::cout << "Target RPY (deg): " << (target.R.eulerAngles(0, 1, 2) * 180.0 / M_PI).transpose()
              << "\n";

    auto result = solveIK(robot, target, 500);
    if (result != IKResult::Success)
    {
      std::cout << "IK failed\n";
      continue;
    }

    auto reached = forwardKinematics(robot);

    std::cout << "Reached position: " << reached.p.transpose() << "\n";

    std::cout << "Reached RPY (deg): "
              << (reached.R.eulerAngles(0, 1, 2) * 180.0 / M_PI).transpose() << "\n";

    std::cout << "Position error: " << (reached.p - target.p).norm() << "\n";
  }
}
