#include "ros/ros.h"
#include "server_slam/RobotHandle.h"
#include "server_slam/SlamServer.h"

RobotHandle::RobotHandle() :
    RobotHandle(10)
{
}

RobotHandle::RobotHandle(int period) :
    upstream_period_(period)
{
  upstream_cl_ = rob_node_.serviceClient<server_slam::requestSeenCells>("map_upstream");
  upstream_thread_ = std::thread(&RobotHandle::upstreamLoop, this);
}

RobotHandle::RobotHandle(int period, const ros::NodeHandle& node) :
    upstream_period_(period), rob_node_(node), upstream_cl_(
        rob_node_.serviceClient<server_slam::requestSeenCells>("map_upstream")), upstream_thread_(
        &RobotHandle::upstreamLoop, this)
{
}

RobotHandle::~RobotHandle()
{
  upstream_thread_.join();
}

void RobotHandle::setServer(std::shared_ptr<SlamServer> server_ptr)
{
  server_ptr_ = server_ptr;
}

void RobotHandle::upstreamLoop()
{
  while (ros::ok())
  {
    ROS_INFO("I will send a request");
    this->sendUpstreamRequest();
    ros::Duration(upstream_period_).sleep();
  }
}

void RobotHandle::sendUpstreamRequest()
{
  upstream_srv_.request.a = 1;

  if (upstream_cl_.call(upstream_srv_))
  {
    ROS_INFO("I got a response");

    if (auto shared = server_ptr_.lock())
    {
      shared->updateMapSegment(upstream_srv_.response);
    }
    else
    {
      ROS_INFO("Unable to lock weak_ptr");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}
