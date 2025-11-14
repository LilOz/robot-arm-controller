#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot
{
inline double degreesToRadians(const double degrees)
{
  constexpr double pi_over_180 = M_PI / 180.0;
  return degrees * pi_over_180;
}

enum class RotationAxis
{
  X,
  Y,
  Z
};

inline std::string rotationAxisToString(RotationAxis axis)
{
  switch (axis)
  {
  case RotationAxis::X:
    return "X";
  case RotationAxis::Y:
    return "Y";
  case RotationAxis::Z:
    return "Z";
  }
  return "Unknown";
}

struct RotationLimits
{
  double min_angle; // in radians
  double max_angle; // in radians
};

struct Link
{
  Link(const std::string& name_, double length_,  RotationAxis axis_, double initial_angle_ = 0.0)
      : name(name_), length(length_), angle(initial_angle_), axis(axis_)
  {
    if (length_ <= 0.0)
    {
      throw std::invalid_argument("Link length must be positive.");
    }
  }

  void setAngle(double angle_rad)
  {
    if (angle_rad < rotation_limits.min_angle)
      angle = rotation_limits.min_angle;
    else if (angle_rad > rotation_limits.max_angle)
      angle = rotation_limits.max_angle;
    else
      angle = angle_rad;
  }

  void rotateBy(double delta_rad)
  {
    setAngle(angle + delta_rad);
  }

  std::string    name;
  double         length;
  RotationLimits rotation_limits{0.0, 0.0}; // in radians
  double         angle;
  RotationAxis   axis;
};

struct Robot
{
  std::string       name;
  std::vector<Link> links;

  bool loadFromJson(const std::string& filename);
};
void printRobot(const Robot& r);
} // namespace robot
