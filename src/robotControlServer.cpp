#include "robotControlServer.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace robot::control
{

// ============ RobotControlSession Implementation ============

RobotControlSession::RobotControlSession(asio::ip::tcp::socket socket, model::Robot& robot,
                                         std::mutex& robot_mutex)
    : socket_(std::move(socket)), robot_(robot), robot_mutex_(robot_mutex)
{
}

void RobotControlSession::start()
{
  doRead();
}

void RobotControlSession::doRead()
{
  auto self(shared_from_this());

  socket_.async_read_some(asio::buffer(data_, max_length),
                          [this, self](std::error_code ec, std::size_t length)
                          {
                            if (!ec)
                            {
                              std::string command(data_, length);

                              // Remove trailing newlines/whitespace
                              command.erase(command.find_last_not_of(" \n\r\t") + 1);

                              handleCommand(command);

                              // Send response
                              std::string response = serializeState();
                              doWrite(response);

                              // Continue reading
                              doRead();
                            }
                            else if (ec != asio::error::eof)
                            {
                              std::cerr << "Read error: " << ec.message() << std::endl;
                            }
                          });
}

void RobotControlSession::doWrite(const std::string& message)
{
  auto self(shared_from_this());

  asio::async_write(socket_, asio::buffer(message),
                    [this, self](std::error_code ec, std::size_t /*length*/)
                    {
                      if (ec)
                      {
                        std::cerr << "Write error: " << ec.message() << std::endl;
                      }
                    });
}

void RobotControlSession::handleCommand(const std::string& command)
{
  std::istringstream iss(command);
  std::string        cmd_type;
  std::getline(iss, cmd_type, '|');

  std::lock_guard<std::mutex> lock(robot_mutex_);

  if (cmd_type == "MOVE_REL")
  {
    double x, y, z;
    char   delim;
    iss >> x >> delim >> y >> delim >> z;

    auto                  current = kinematics::forwardKinematics(robot_);
    kinematics::Transform target;
    target.p = current.p + Eigen::Vector3d(x, y, z);
    target.R = current.R;

    auto result = kinematics::solveIK(robot_, target, 500, 0.01);
    std::cout << "Move relative (" << x << ", " << y << ", " << z
              << "): " << (result == kinematics::IKResult::Success ? "SUCCESS" : "FAILED")
              << std::endl;
  }
  else if (cmd_type == "MOVE_ABS")
  {
    double x, y, z;
    char   delim;
    iss >> x >> delim >> y >> delim >> z;

    auto                  current = kinematics::forwardKinematics(robot_);
    kinematics::Transform target;
    target.p = Eigen::Vector3d(x, y, z);
    target.R = current.R;

    auto result = kinematics::solveIK(robot_, target, 500, 0.01);
    std::cout << "Move absolute (" << x << ", " << y << ", " << z
              << "): " << (result == kinematics::IKResult::Success ? "SUCCESS" : "FAILED")
              << std::endl;
  }
  else if (cmd_type == "ROTATE")
  {
    double ax, ay, az, angle;
    char   delim;
    iss >> ax >> delim >> ay >> delim >> az >> delim >> angle;

    auto            current = kinematics::forwardKinematics(robot_);
    Eigen::Matrix3d R_delta =
        Eigen::AngleAxisd(angle * M_PI / 180.0, Eigen::Vector3d(ax, ay, az).normalized())
            .toRotationMatrix();

    kinematics::Transform target;
    target.p = current.p;
    target.R = current.R * R_delta;

    auto result = kinematics::solveIK(robot_, target, 500, 0.01);
    std::cout << "Rotate: " << (result == kinematics::IKResult::Success ? "SUCCESS" : "FAILED")
              << std::endl;
  }
  else if (cmd_type == "SET_JOINTS")
  {
    std::string angles_str;
    std::getline(iss, angles_str);

    std::istringstream  angles_iss(angles_str);
    std::vector<double> angles;
    double              angle;
    char                delim;

    while (angles_iss >> angle)
    {
      angles.push_back(angle * M_PI / 180.0); // Convert to radians
      angles_iss >> delim;                    // Skip comma
    }

    if (angles.size() == robot_.links.size())
    {
      for (size_t i = 0; i < robot_.links.size(); ++i)
        robot_.links[i].angle = angles[i];

      std::cout << "Set joints: SUCCESS" << std::endl;
    }
    else
    {
      std::cout << "Set joints: FAILED (wrong number of angles)" << std::endl;
    }
  }
  else if (cmd_type == "RESET")
  {
    robot_.resetLinkAngles();

    std::cout << "Robot reset to home position" << std::endl;
  }
  else if (cmd_type == "GET_STATE")
  {
    // State will be sent in response
  }
  else
  {
    std::cout << "Unknown command: " << cmd_type << std::endl;
  }
}

std::string RobotControlSession::serializeState()
{
  auto T = kinematics::forwardKinematics(robot_);
  auto RPY = T.R.eulerAngles(0, 1, 2) * 180.0 / M_PI;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "STATE|";

  // Position
  oss << T.p.x() << "," << T.p.y() << "," << T.p.z() << "|";

  // Orientation (RPY in degrees)
  oss << RPY[0] << "," << RPY[1] << "," << RPY[2] << "|";

  // Joint angles (in degrees)
  for (size_t i = 0; i < robot_.links.size(); ++i)
  {
    oss << robot_.links[i].angle * 180.0 / M_PI;
    if (i < robot_.links.size() - 1)
      oss << ",";
  }

  return oss.str();
}

// ============ RobotControlServer Implementation ============

RobotControlServer::RobotControlServer(model::Robot& robot, unsigned short port)
    : robot_(robot), port_(port),
      acceptor_(io_context_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)), running_(false)
{
}

RobotControlServer::~RobotControlServer()
{
  stop();
}

void RobotControlServer::start()
{
  if (running_)
    return;

  running_ = true;

  std::cout << "Robot control server starting on port " << port_ << std::endl;

  doAccept();

  // Run io_context in separate thread
  io_thread_ = std::thread([this]() { runIOContext(); });

  std::cout << "Robot control server running" << std::endl;
}

void RobotControlServer::stop()
{
  if (!running_)
    return;

  running_ = false;

  io_context_.stop();

  if (io_thread_.joinable())
    io_thread_.join();

  std::cout << "Robot control server stopped" << std::endl;
}

void RobotControlServer::doAccept()
{
  acceptor_.async_accept(
      [this](std::error_code ec, asio::ip::tcp::socket socket)
      {
        if (!ec)
        {
          std::cout << "Client connected from " << socket.remote_endpoint().address().to_string()
                    << std::endl;

          std::make_shared<RobotControlSession>(std::move(socket), robot_, robot_mutex_)->start();
        }
        else
        {
          std::cerr << "Accept error: " << ec.message() << std::endl;
        }

        // Continue accepting connections
        if (running_)
          doAccept();
      });
}

void RobotControlServer::runIOContext()
{
  try
  {
    io_context_.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "IO context exception: " << e.what() << std::endl;
  }
}

bool RobotControlServer::moveRelative(const Eigen::Vector3d& delta)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  auto current = kinematics::forwardKinematics(robot_);

  kinematics::Transform target;
  target.p = current.p + delta;
  target.R = current.R;

  auto result = kinematics::solveIK(robot_, target, 500, 0.01);
  return result == kinematics::IKResult::Success;
}

bool RobotControlServer::moveAbsolute(const Eigen::Vector3d& target_pos)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  auto current = kinematics::forwardKinematics(robot_);

  kinematics::Transform target;
  target.p = target_pos;
  target.R = current.R;

  auto result = kinematics::solveIK(robot_, target, 500, 0.01);
  return result == kinematics::IKResult::Success;
}

bool RobotControlServer::rotateRelative(const Eigen::Vector3d& axis, double angle)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  auto current = kinematics::forwardKinematics(robot_);

  Eigen::Matrix3d R_delta = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();

  kinematics::Transform target;
  target.p = current.p;
  target.R = current.R * R_delta;

  auto result = kinematics::solveIK(robot_, target, 500, 0.01);
  return result == kinematics::IKResult::Success;
}

void RobotControlServer::getState(kinematics::Transform& current_pose,
                                  std::vector<double>&   joint_angles)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  current_pose = kinematics::forwardKinematics(robot_);

  joint_angles.clear();
  for (const auto& link : robot_.links)
    joint_angles.push_back(link.angle);
}

void RobotControlServer::reset()
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  robot_.resetLinkAngles();
}

} // namespace robot::control
