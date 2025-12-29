#include "kinematics.hpp"
#include "robotModel.hpp"
#include <fstream>
#include <iomanip>
#include <limits>

namespace robot::kinematics
{

// Rotation matrix for an axis + angle (radians)
inline auto rotationMatrix(Eigen::Vector3d axis, double angle)
{
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

// Composition of transforms: C = A ∘ B
inline Transform operator*(const Transform& A, const Transform& B)
{
  Transform C;
  C.R = A.R * B.R;
  C.p = A.R * B.p + A.p;
  return C;
}

inline Transform linkTransform(const model::Link& link)
{
  Eigen::Matrix3d R = rotationMatrix(link.axis, link.angle);
  Eigen::Vector3d offsetLocal(0, 0, link.length);
  auto            p = R * offsetLocal;
  return Transform{R, p};
}

inline Eigen::Vector3d orientationError(const Eigen::Matrix3d& R_current,
                                        const Eigen::Matrix3d& R_target)
{
  Eigen::Matrix3d   R_err = R_target * R_current.transpose();
  Eigen::AngleAxisd aa(R_err);

  double angle = aa.angle();
  if (angle > M_PI)
    angle -= 2.0 * M_PI;

  if (std::abs(angle) < 1e-9)
    return Eigen::Vector3d::Zero();

  return aa.axis() * angle;
}

Transform forwardKinematics(const model::Robot& robot)
{
  Transform T;
  for (const auto& link : robot.links)
  {
    Transform Ti = linkTransform(link);
    T = T * Ti;
  }
  return T;
}

std::vector<Transform> forwardKinematicsAll(const model::Robot& robot)
{
  std::vector<Transform> transforms;
  transforms.reserve(robot.links.size() + 1);

  Transform T;
  transforms.push_back(T);

  for (const auto& link : robot.links)
  {
    Transform Ti = linkTransform(link);
    T = T * Ti;
    transforms.push_back(T);
  }

  return transforms;
}

// Full 6DOF Jacobian (position + orientation)
Eigen::MatrixXd computeJacobian6DOF(const model::Robot& robot)
{
  size_t          n = robot.links.size();
  Eigen::MatrixXd J(6, n);

  const auto Ts = forwardKinematicsAll(robot);
  const auto pe = Ts.back().p;

  for (size_t i = 0; i < n; ++i)
  {
    const auto&     Ti = Ts[i];
    Eigen::Vector3d zi = Ti.R * robot.links[i].axis;
    Eigen::Vector3d pi = Ti.p;

    // Linear velocity (position) contribution
    J.block<3, 1>(0, i) = zi.cross(pe - pi);

    // Angular velocity (orientation) contribution
    J.block<3, 1>(3, i) = zi;
  }

  return J;
}

// Adaptive damping based on error magnitude
double adaptiveLambda(double error, double baseλ = 0.01)
{
  // Increase damping when error is large (more stable)
  // Decrease when error is small (faster convergence)
  if (error > 0.1)
    return baseλ * 10.0;
  else if (error > 0.01)
    return baseλ * 2.0;
  else
    return baseλ;
}

IKResult solveIK(model::Robot& robot, const Transform& target, int iterations, double lambda)
{
  if (target.p.norm() > robot.totalLength)
    return IKResult::Unreachable;

  double    prevError = std::numeric_limits<double>::max();
  int       stagnantCount = 0;
  const int maxStagnant = 20;
  auto      prevAngles = robot.getLinkAngles();

  for (int k = 0; k < iterations; ++k)
  {
    // Current end-effector pose
    auto T_current = forwardKinematics(robot);

    // Position error
    Eigen::Vector3d pos_error = target.p - T_current.p;

    // Orientation error
    Eigen::Vector3d rot_error = orientationError(T_current.R, target.R);

    // Combined 6D error vector
    Eigen::VectorXd error(6);
    error << pos_error, rot_error;

    double totalError = error.norm();

    // Check convergence
    if (totalError < 1e-4)
      return IKResult::Success;

    // Improved divergence detection: allow temporary increases
    if (totalError > prevError * 1.2) // More lenient threshold
    {
      stagnantCount++;
      if (stagnantCount > maxStagnant)
      {
        robot.setLinkAngles(prevAngles); // revert to last good angles
        return IKResult::Diverged;
      }
    }
    else
    {
      stagnantCount = 0; // Reset if making progress
    }

    prevError = totalError;

    // Adaptive damping
    double adaptLambda = adaptiveLambda(totalError, lambda);

    // Compute full 6DOF Jacobian
    Eigen::MatrixXd J = computeJacobian6DOF(robot);

    // Damped least squares
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd lambdaI = adaptLambda * adaptLambda * Eigen::MatrixXd::Identity(6, 6);

    Eigen::VectorXd dq = J.transpose() * (JJt + lambdaI).inverse() * error;

    // Apply joint updates with step size limiting
    double maxStep = 0.5; // radians per iteration
    if (dq.norm() > maxStep)
      dq = dq * (maxStep / dq.norm());

    for (size_t i = 0; i < robot.links.size(); ++i)
      robot.links[i].rotateBy(dq[i]);
  }

  robot.setLinkAngles(prevAngles); // revert to last good angles
  return IKResult::MaxIterationsExceeded;
}

void exportForwardKinematics(const model::Robot& robot, const std::string& filename)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("Could not open file for writing FK data");
  }

  file << "x,y,z,roll,pitch,yaw\n";

  Transform T;
  T.p = Eigen::Vector3d::Zero();

  for (const auto& link : robot.links)
  {
    Transform Ti = linkTransform(link);
    T = T * Ti;

    file << std::fixed << std::setprecision(6) << T.p.x() << "," << T.p.y() << "," << T.p.z();
    auto RPY = T.R.eulerAngles(0, 1, 2) * 180 / M_PI;
    file << "," << RPY[0] << "," << RPY[1] << "," << RPY[2];
    file << "\n";
  }

  file.close();
}

} // namespace robot::kinematics
