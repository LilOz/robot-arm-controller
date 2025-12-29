#pragma once

#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace robot::model
{
inline double degreesToRadians(const double degrees)
{
  constexpr double pi_over_180 = M_PI / 180.0;
  return degrees * pi_over_180;
}

struct RotationLimits
{
  double min_angle; // in radians
  double max_angle; // in radians
};

struct Link
{
  Link(const std::string& name_, double length_, Eigen::Vector3d axis_, double initialAngle_ = 0.0)
      : name(name_), length(length_), angle(initialAngle_), initialAngle(initialAngle_), axis(axis_)
  {
    if (length_ < 0.0)
    {
      throw std::invalid_argument("Link length must be positive.");
    }
  }

  void setAngle(double angle_rad)
  {
    if (angle_rad < rotationLimits.min_angle)
      angle = rotationLimits.min_angle;
    else if (angle_rad > rotationLimits.max_angle)
      angle = rotationLimits.max_angle;
    else
      angle = angle_rad;
  }

  void rotateBy(double delta_rad)
  {
    setAngle(angle + delta_rad);
  }

  void resetAngle()
  {
    setAngle(initialAngle);
  }

  std::string     name;
  double          length;
  RotationLimits  rotationLimits{0.0, 0.0}; // in radians
  double          angle;
  double          initialAngle;
  Eigen::Vector3d axis; // should be one of the unit vectors
};

struct Robot
{
  explicit Robot(const std::string& filename)
  {
    if (!loadFromJson(filename))
    {
      throw std::runtime_error("Failed to load robot model from file: " + filename);
    }
  }

  std::string       name;
  std::vector<Link> links;
  double            totalLength = 0.0;

  bool loadFromJson(const std::string& filename);

  void setLinkAngles(const std::vector<double>& angles_rad)
  {
    if (angles_rad.size() != links.size())
    {
      throw std::invalid_argument("Number of angles does not match number of links.");
    }
    for (size_t i = 0; i < links.size(); ++i)
    {
      links[i].setAngle(angles_rad[i]);
    }
  }

  std::vector<double> getLinkAngles() const
  {
    std::vector<double> angles;
    angles.reserve(links.size());
    for (const auto& link : links)
    {
      angles.push_back(link.angle);
    }
    return angles;
  }

  void resetLinkAngles()
  {
    for (auto& link : links)
    {
      link.resetAngle();
    }
  }
};
void printRobot(const Robot& r);
} // namespace robot::model
