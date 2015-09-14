#ifndef SERVER_SLAM_ROBOTHANDLE_H
#define SERVER_SLAM_ROBOTHANDLE_H

#include "ros/ros.h"
#include <memory>
#include <thread>
#include "server_slam/requestSeenCells.h"
#include "server_slam/PointAccumulator.h"

class SlamServer;

class RobotHandle
{
public:

  RobotHandle();
  RobotHandle(int period);
  RobotHandle(int period, const ros::NodeHandle & node, const std::string name);
  ~RobotHandle();
  void setServer(std::shared_ptr<SlamServer> server_ptr);

private:

  ros::NodeHandle rob_node_;
  ros::ServiceClient upstream_cl_;
  server_slam::requestSeenCells upstream_srv_;
  std::thread upstream_thread_;
  std::weak_ptr<SlamServer> server_ptr_;
  std::string name_;

  int upstream_period_;
  void upstreamLoop();
  void sendUpstreamRequest();
};

#endif /* ifndef SERVER_SLAM_ROBOTHANDLE_H */
