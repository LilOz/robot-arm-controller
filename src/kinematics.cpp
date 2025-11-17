#include "kinematics.hpp"
#include "robotModel.hpp"
#include <fstream>
#include <iomanip>
#include <limits>

namespace robot
{

// Rotation matrix for an axis + angle (radians)
inline auto rotationMatrix(Eigen::Vector3d axis, double angle)
{
  auto u = axis.normalized();

  double x = u.x();
  double y = u.y();
  double z = u.z();

  double c = std::cos(angle);
  double s = std::sin(angle);

  double one_c = 1.0 - c;

  Eigen::Matrix3d R;
  R << c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, y * x * one_c + z * s,
      c + y * y * one_c, y * z * one_c - x * s, z * x * one_c - y * s, z * y * one_c + x * s,
      c + z * z * one_c;

  return R;
}

// Composition of transforms: C = A ∘ B
// T maps a point expressed in B's (child) frame to A's frame (parent)
inline Transform operator*(const Transform& A, const Transform& B)
{
  Transform C;
  C.R = A.R * B.R;
  C.p = A.R * B.p + A.p;
  return C;
}

inline Transform linkTransform(const Link& link)
{
  Eigen::Matrix3d R = rotationMatrix(link.axis, link.angle);
  // offset along this link's local Z, expressed in the *parent* frame
  Eigen::Vector3d offsetLocal(0, 0, link.length);

  auto p = R * offsetLocal;

  return Transform{R, p};
}

Transform forwardKinematics(const Robot& robot)
{
  Transform T; // identity: R = I, p = 0

  for (const auto& link : robot.links)
  {
    Transform Ti = linkTransform(link);
    T = T * Ti;
  }

  return T; // end-effector in base frame
}

std::vector<Transform> forwardKinematicsAll(const Robot& robot)
{
  std::vector<Transform> transforms;
  transforms.reserve(robot.links.size() + 1);

  Transform T;             // identity
  transforms.push_back(T); // base frame

  for (const auto& link : robot.links)
  {
    Transform Ti = linkTransform(link);
    T = T * Ti;
    transforms.push_back(T);
  }

  return transforms;
}

Eigen::MatrixXd computeJacobian(const Robot& robot)
{
  auto n = robot.links.size();

  Eigen::MatrixXd J(3, n);

  const auto       Ts = forwardKinematicsAll(robot);
  const auto pe = Ts.back().p;

  for (size_t i = 0; i < n; ++i)
  {
    const auto& Ti = Ts[i];

    // world joint axis
    Eigen::Vector3d zi = Ti.R * robot.links[i].axis;

    // world joint position
    Eigen::Vector3d pi = Ti.p;

    // Jacobian column
    J.col(i) = zi.cross(pe - pi);
  }

  return J;
}

Eigen::VectorXd ikStep(const Robot& robot, const Eigen::Vector3d& target, const Eigen::Vector3d& pe,
                       const Eigen::Vector3d& error, double lambda)
{

  Eigen::MatrixXd J = computeJacobian(robot);

  // Damped least squares
  Eigen::MatrixXd JJt = J * J.transpose();
  Eigen::Matrix3d lambdaI = lambda * lambda * Eigen::Matrix3d::Identity();

  Eigen::VectorXd dq = J.transpose() * (JJt + lambdaI).inverse() * error;
  return dq;
}

IKResult solveIK(Robot& robot, const Eigen::Vector3d& target, int iterations)
{
  if (target.norm() > robot.totalLength)
    return IKResult::Unreachable;

  double prevError = std::numeric_limits<double>::max();
  for (int k = 0; k < iterations; ++k)
  {
    auto            pe = forwardKinematics(robot).p;
    Eigen::Vector3d error = target - pe;

    if (error.norm() > prevError * 1.05)
      return IKResult::Diverged;
    prevError = error.norm();

    Eigen::VectorXd dq = ikStep(robot, target, pe, error, 0.1);

    for (size_t i = 0; i < robot.links.size(); ++i)
      robot.links[i].rotateBy(dq[i]);

    if (error.norm() < 1e-4)
      return IKResult::Success;
  }

  return IKResult::MaxIterationsExceeded;
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

    Transform Ti = linkTransform(link);
    T = T * Ti;

    // Write position and orientation (RPY)
    file << std::fixed << std::setprecision(6) << T.p.x() << "," << T.p.y() << "," << T.p.z();
    auto RPY = T.R.eulerAngles(0, 1, 2) * 180 / M_PI;
    file << "," << RPY[0] << "," << RPY[1] << "," << RPY[2];
    file << "\n";
  }

  file.close();
}

} // namespace robot
