// robotControlServer.hpp
#pragma once

#include "kinematics.hpp"
#include "robotModel.hpp"
#include <asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

namespace robot::control
{

class RobotControlSession : public std::enable_shared_from_this<RobotControlSession>
{
public:
  RobotControlSession(asio::ip::tcp::socket socket, model::Robot& robot, std::mutex& robot_mutex);
  
  void start();
  
private:
  void doRead();
  void doWrite(const std::string& message);
  void handleCommand(const std::string& command);
  std::string serializeState();
  
  asio::ip::tcp::socket socket_;
  model::Robot& robot_;
  std::mutex& robot_mutex_;
  
  enum { max_length = 1024 };
  char data_[max_length];
};

class RobotControlServer
{
public:
  RobotControlServer(model::Robot& robot, unsigned short port = 8888);
  ~RobotControlServer();
  
  void start();
  void stop();
  
  // Programmatic control methods (thread-safe)
  bool moveRelative(const Eigen::Vector3d& delta);
  bool moveAbsolute(const Eigen::Vector3d& target);
  bool rotateRelative(const Eigen::Vector3d& axis, double angle);
  void getState(kinematics::Transform& current_pose, std::vector<double>& joint_angles);
  void reset();
  
private:
  void doAccept();
  void runIOContext();
  
  model::Robot& robot_;
  std::mutex robot_mutex_;
  unsigned short port_;
  
  asio::io_context io_context_;
  asio::ip::tcp::acceptor acceptor_;
  std::thread io_thread_;
  
  std::atomic<bool> running_;
};

} // namespace robot::control
