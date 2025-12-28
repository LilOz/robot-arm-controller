#include "robotModel.hpp"
#include "kinematics.hpp"
#include "robotControlServer.hpp"
#include <iostream>

int main()
{
  using namespace robot;

  try
  {
    // Load robot model
    model::Robot robot("config/models/6dof_spherical_model.json");

    // Create control server
    control::RobotControlServer server(robot, 8888);

    // Start server
    server.start();

    std::cout << "\n=== Robot Control Server ===" << std::endl;
    std::cout << "Server listening on port 8888" << std::endl;
    std::cout << "Connect from Blender to control the robot" << std::endl;
    std::cout << "\nLocal keyboard controls:" << std::endl;
    std::cout << "  WASD - move in XY plane" << std::endl;
    std::cout << "  Q/E  - move up/down" << std::endl;
    std::cout << "  R    - rotate around X" << std::endl;
    std::cout << "  T    - rotate around Y" << std::endl;
    std::cout << "  Y    - rotate around Z" << std::endl;
    std::cout << "  H    - reset to home" << std::endl;
    std::cout << "  P    - print current state" << std::endl;
    std::cout << "  X    - exit" << std::endl;
    std::cout << "\nWaiting for commands...\n" << std::endl;

    // Local control loop
    char input;
    bool running = true;

    while (running && std::cin >> input)
    {
      double step = 0.05;                    // 5cm movements
      double rot_step = 15.0 * M_PI / 180.0; // 15 degree rotations

      switch (input)
      {
      case 'w':
      case 'W':
        if (server.moveRelative(Eigen::Vector3d(step, 0, 0)))
          std::cout << "→ Moved forward" << std::endl;
        else
          std::cout << "✗ Move failed" << std::endl;
        break;

      case 's':
      case 'S':
        if (server.moveRelative(Eigen::Vector3d(-step, 0, 0)))
          std::cout << "→ Moved backward" << std::endl;
        else
          std::cout << "✗ Move failed" << std::endl;
        break;

      case 'a':
      case 'A':
        if (server.moveRelative(Eigen::Vector3d(0, -step, 0)))
          std::cout << "→ Moved left" << std::endl;
        else
          std::cout << "✗ Move failed" << std::endl;
        break;

      case 'd':
      case 'D':
        if (server.moveRelative(Eigen::Vector3d(0, step, 0)))
          std::cout << "→ Moved right" << std::endl;
        else
          std::cout << "✗ Move failed" << std::endl;
        break;

      case 'q':
      case 'Q':
        if (server.moveRelative(Eigen::Vector3d(0, 0, step)))
          std::cout << "→ Moved up" << std::endl;
        else
          std::cout << "✗ Move failed" << std::endl;
        break;

      case 'e':
      case 'E':
        if (server.moveRelative(Eigen::Vector3d(0, 0, -step)))
          std::cout << "→ Moved down" << std::endl;
        else
          std::cout << "✗ Move failed" << std::endl;
        break;

      case 'r':
      case 'R':
        if (server.rotateRelative(Eigen::Vector3d::UnitX(), rot_step))
          std::cout << "→ Rotated around X" << std::endl;
        else
          std::cout << "✗ Rotation failed" << std::endl;
        break;

      case 't':
      case 'T':
        if (server.rotateRelative(Eigen::Vector3d::UnitY(), rot_step))
          std::cout << "→ Rotated around Y" << std::endl;
        else
          std::cout << "✗ Rotation failed" << std::endl;
        break;

      case 'y':
      case 'Y':
        if (server.rotateRelative(Eigen::Vector3d::UnitZ(), rot_step))
          std::cout << "→ Rotated around Z" << std::endl;
        else
          std::cout << "✗ Rotation failed" << std::endl;
        break;

      case 'h':
      case 'H':
        server.reset();
        std::cout << "→ Reset to home position" << std::endl;
        break;

      case 'p':
      case 'P':
      {
        kinematics::Transform pose;
        std::vector<double>   joints;
        server.getState(pose, joints);

        std::cout << "\n=== Current State ===" << std::endl;
        std::cout << "Position: " << pose.p.transpose() << std::endl;

        auto RPY = pose.R.eulerAngles(0, 1, 2) * 180.0 / M_PI;
        std::cout << "Orientation (RPY deg): " << RPY.transpose() << std::endl;

        std::cout << "Joint angles (deg): ";
        for (size_t i = 0; i < joints.size(); ++i)
        {
          std::cout << joints[i] * 180.0 / M_PI;
          if (i < joints.size() - 1)
            std::cout << ", ";
        }
        std::cout << "\n" << std::endl;
        break;
      }

      case 'x':
      case 'X':
        running = false;
        break;

      default:
        std::cout << "Unknown command: " << input << std::endl;
      }
    }

    std::cout << "\nShutting down..." << std::endl;
    server.stop();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
