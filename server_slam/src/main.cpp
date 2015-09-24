#include "ros/ros.h"
#include <memory>

#include "server_slam/SlamServer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_server");

  // try to load map from server

  if (ros::console::set_logger_level("ros.server_slam.init_robot", ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

//  if (ros::console::set_logger_level("ros.server_slam.mutex", ros::console::levels::Debug))
//  {
//    ros::console::notifyLoggerLevelsChanged();
//  }
//
//  if (ros::console::set_logger_level("ros.server_slam.load_map", ros::console::levels::Debug))
//  {
//    ros::console::notifyLoggerLevelsChanged();
//  }

  ros::NodeHandle n;

  ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("static_map");

  nav_msgs::GetMap srv;

  std::shared_ptr<SlamServer> server;

  if (map_client.call(srv))
  {
    server = std::make_shared<SlamServer>(srv.response);
  }
  else
  {
    server = std::make_shared<SlamServer>();
  }



  ros::spin();

  return (0);
}
