#include "ros/ros.h"
#include <memory>

#include "server_slam/SlamServer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_server");

  auto server = std::make_shared<SlamServer>();
  server->registerRobot("Robot 1");

  ros::spin();

  return (0);
}
