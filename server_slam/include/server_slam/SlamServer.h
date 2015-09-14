#ifndef SERVER_SLAM_SLAMSERVER_H
#define SERVER_SLAM_SLAMSERVER_H

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"

#include <forward_list>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <unordered_map>

#include "gmapping/scanmatcher/smmap.h"
#include "server_slam/RobotHandle.h"
#include "server_slam/requestSeenCells.h"
#include "server_slam/turnOff.h"
#include "server_slam/poseRequest.h"

class SlamServer : public std::enable_shared_from_this<SlamServer>

{

public:
  SlamServer();
  ~SlamServer();
  void registerRobot(const std::string name);
  void updateMapSegment(server_slam::requestSeenCells::Response& response);
  void publishMapLoop();
  void publishMap();
  bool mapDownload(server_slam::requestSeenCells::Request & req,
                   server_slam::requestSeenCells::Response& res);
  bool robotTurnOff(server_slam::turnOff::Request & req,
                    server_slam::turnOff::Response& res);
  bool poseRequest(server_slam::poseRequest::Request & req,
                    server_slam::poseRequest::Response& res);


private:
  ros::NodeHandle server_node_;
  ros::Publisher map_pub_;
  ros::Duration publish_period_;
  ros::ServiceServer map_down_;
  ros::ServiceServer robot_turn_off_;
  ros::ServiceServer register_robot_;
  nav_msgs::GetMap::Response map_msg_;
  tf::TransformListener tf_;

  GMapping::ScanMatcherMap map_;
  std::mutex map_mutex_;
  std::mutex saved_robots_mutex_;
  std::thread map_publish_thread_;
  std::forward_list<std::shared_ptr<RobotHandle>> rob_list_;
  std::unordered_map<std::string, GMapping::OrientedPoint> saved_robots_;
  int robot_count_;
  std::string map_frame_;

  server_slam::requestSeenCells::Response buffer_;

};

#endif
