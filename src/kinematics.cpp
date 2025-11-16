#include "kinematics.hpp"
#include <Eigen/Dense>
#include <iomanip>
#include "robotModel.hpp"
#include <fstream>

namespace robot
{

// Rotation matrix for an axis + angle (radians)
inline Eigen::Matrix3d rotationMatrix(RotationAxis axis, double angle)
{
  double c = std::cos(angle);
  double s = std::sin(angle);

  switch (axis)
  {
  case RotationAxis::X:
    return (Eigen::Matrix3d() << 1, 0, 0, 0, c, -s, 0, s, c).finished();

  case RotationAxis::Y:
    return (Eigen::Matrix3d() << c, 0, s, 0, 1, 0, -s, 0, c).finished();

  case RotationAxis::Z:
    return (Eigen::Matrix3d() << c, -s, 0, s, c, 0, 0, 0, 1).finished();
  }

  return Eigen::Matrix3d::Identity(); // fallback
}

// Composition of transforms: C = A ∘ B
inline Transform operator*(const Transform& A, const Transform& B)
{
  Transform C;
  C.R = A.R * B.R;
  C.p = C.R * B.p + A.p; // C.R * B.p is B's translation in the world frame
  return C;              // apply A then B
}

// --------------------------
// Forward Kinematics
// --------------------------

Transform forwardKinematics(const Robot& robot)
{
  Transform T;
  T.p = Eigen::Vector3d(0, 0, 0); // start at origin

  for (const auto& link : robot.links)
  {
    // Rotation matrix for this joint
    Eigen::Matrix3d R = rotationMatrix(link.axis, link.angle);

    // Translation along this link (assumed along local Z axis)
    Eigen::Vector3d p(0, 0, link.length);

    Transform Ti{R, p};

    // apply this link's transform
    T = T * Ti;
  }

  return T; // end-effector transform in world frame
}

void exportForwardKinematics(const Robot& robot, const std::string& filename)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("Could not open file for writing FK data");
  }

  // CSV header
  file << "x,y,z,roll,pitch,yaw\n";

  Transform T;
  T.p = Eigen::Vector3d::Zero();

  for (const auto& link : robot.links)
  {
    Eigen::Matrix3d R = rotationMatrix(link.axis, link.angle);
    Eigen::Vector3d p(0, 0, link.length);
    Transform       Ti{R, p};

    T = T * Ti;

    // Write position and orientation (RPY)
    file << std::fixed << std::setprecision(6) << T.p.x() << "," << T.p.y() << "," << T.p.z();
    auto RPY = R.eulerAngles(0, 1, 2) * 180 / M_PI;
    file << "," << RPY[0] << "," << RPY[1] << "," << RPY[2];
    file << "\n";
  }

  file.close();
}
} // namespace robot
