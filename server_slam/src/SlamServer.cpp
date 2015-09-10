#include "ros/ros.h"

#include "server_slam/SlamServer.h"
#include <gmapping/utils/point.h>

// linear index
#define MAP_IDX(width, x, y) ((width) * (y) + (x))

SlamServer::SlamServer() :
    map_(GMapping::Point(0, 0), -20, -20, 20, 20, 0.05), server_node_(), robot_count_(0), publish_period_(5), map_frame_(
        "map"), map_pub_(server_node_.advertise<nav_msgs::OccupancyGrid>("server_map", 1, true))
{
  map_msg_.map.info.resolution = 0.05;
  map_msg_.map.info.origin.position.x = 0.0;
  map_msg_.map.info.origin.position.y = 0.0;
  map_msg_.map.info.origin.position.z = 0.0;
  map_msg_.map.info.origin.orientation.x = 0.0;
  map_msg_.map.info.origin.orientation.y = 0.0;
  map_msg_.map.info.origin.orientation.z = 0.0;
  map_msg_.map.info.origin.orientation.w = 1.0;

  map_msg_.map.info.width = map_.getMapSizeX();
  map_msg_.map.info.height = map_.getMapSizeY();
  map_msg_.map.info.origin.position.x = -20;
  map_msg_.map.info.origin.position.y = -20;
  map_msg_.map.data.resize(map_msg_.map.info.width * map_msg_.map.info.height);

  map_publish_thread_ = std::thread(&SlamServer::publishMapLoop, this);

  map_down_ = server_node_.advertiseService("map_download", &SlamServer::mapDownload, this);
}

SlamServer::~SlamServer()
{
  map_publish_thread_.join();
}

void SlamServer::registerRobot()
{
  auto rob = std::make_shared<RobotHandle>(10, server_node_);

  rob->setServer(shared_from_this());
  rob_list_.push_front(rob);
  robot_count_++;
}

void SlamServer::updateMapSegment(server_slam::requestSeenCells::Response& response)
{
  //This is just temporary
  if (static_cast<bool>(response.b.size()))
  {
    buffer_ = response;
  }

  std::lock_guard<std::mutex> guard(map_mutex_);

  ROS_INFO("I try to update my map!");

  for (int i = 0; i < response.b.size(); i++)
  {
    map_.cell(GMapping::IntPoint(response.b[i].point.x, response.b[i].point.y)).setfromMsg(response.b[i].pa);
  }
}

void SlamServer::publishMapLoop()
{
  while (ros::ok())
  {
    ROS_INFO("I will publish my Map!");
    this->publishMap();
    ros::Duration(publish_period_).sleep();
  }
}

void SlamServer::publishMap()
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  for (int x = 0; x < map_.getMapSizeX(); x++)
  {
    for (int y = 0; y < map_.getMapSizeY(); y++)
    {
      double occ = (double)map_.cell(GMapping::IntPoint(x, y));

      if (occ < 0)
      {
        map_msg_.map.data[MAP_IDX(map_msg_.map.info.width, x, y)] = occ;
      }
      else
      {
        map_msg_.map.data[MAP_IDX(map_msg_.map.info.width, x, y)] = occ * 100;
      }
    }
  }

  map_msg_.map.header.stamp = ros::Time::now();
  map_msg_.map.header.frame_id = tf_.resolve(map_frame_);

  map_pub_.publish(map_msg_.map);
}

bool SlamServer::mapDownload(server_slam::requestSeenCells::Request & req, server_slam::requestSeenCells::Response& res)
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  if (req.a == 1)
  {
//    std::cerr << buffer_.b.size() << std::endl;
    res = buffer_;
    return true;
  }
  else
  {
    return false;
  }
}
