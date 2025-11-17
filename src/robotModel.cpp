#include "robotModel.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace robot
{

void printRobot(const Robot& r)
{
  std::cout << "\nRobot name: " << r.name << "\n";

  std::cout << "\nLinks:\n";
  for (const auto& link : r.links)
  {
    std::cout << "  Name: " << link.name << ", Length: " << link.length << ", Angle: " << link.angle
              << "\n";
  }
  std::cout << std::endl;
}

bool Robot::loadFromJson(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "Failed to open JSON file: " << filename << "\n";
    return false;
  }

  json data = json::parse(file);

  // Robot name and base
  name = data.value("name", "unnamed_robot");

  // Clear old links
  links.clear();

  const auto& jlinks = data["links"];
  links.reserve(jlinks.size());
  double total_length = 0.0;

  for (size_t i = 0; i < jlinks.size(); ++i)
  {
    const auto& jl = jlinks[i];

    // Required fields
    std::string link_name = jl["name"];
    double      length = jl["length"];

    // Initial angle (deg → rad)
    double initial_angle_rad = degreesToRadians(jl.value("initial_angle_deg", 0.0));

    // Axis
    std::string     axis_str = jl["axis"];
    Eigen::Vector3d axis;

    if (axis_str == "X")
      axis = Eigen::Vector3d::UnitX();
    else if (axis_str == "Y")
      axis = Eigen::Vector3d::UnitY();
    else if (axis_str == "Z")
      axis = Eigen::Vector3d::UnitZ();
    else
    {
      std::cerr << "Warning: Unknown axis '" << axis_str << "' for link '" << link_name
                << "', defaulting to Z\n";
      return false;
    }

    // Create link
    links.emplace_back(link_name, length, axis, initial_angle_rad);
    total_length += length;

    // Rotation limits (deg → rad)
    auto limits_deg = jl["limits_deg"].get<std::array<double, 2>>();

    std::array<double, 2> limits_rad;
    std::ranges::transform(limits_deg, limits_rad.begin(), degreesToRadians);

    // Convention: (0,0) means unlimited
    if (limits_deg[0] == 0.0 && limits_deg[1] == 0.0)
    {
      limits_rad = {-M_PI, M_PI};
    }

    links.back().rotation_limits = {limits_rad[0], limits_rad[1]};
  }

  std::cout << "Loaded robot model: " << name << " with " << links.size() << " links.\n";

  totalLength = total_length;
  return true;
}

} // namespace robot
