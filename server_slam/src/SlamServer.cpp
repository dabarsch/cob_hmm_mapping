#include "ros/ros.h"

#include "server_slam/SlamServer.h"
#include <gmapping/utils/point.h>

#include <utility>

// linear index
#define MAP_IDX(width, x, y) ((width) * (y) + (x))

SlamServer::SlamServer(const nav_msgs::GetMapResponse& map) :
    map_(
        GMapping::Point(
            map.map.info.origin.position.x
                + 0.5 * map.map.info.resolution * map.map.info.width,
            map.map.info.origin.position.y
                + 0.5 * map.map.info.resolution * map.map.info.height),
        map.map.info.width * map.map.info.resolution,
        map.map.info.height * map.map.info.resolution, map.map.info.resolution), server_node_(), robot_count_(
        0), publish_period_(5), map_frame_("map"), map_pub_(
        server_node_.advertise<nav_msgs::OccupancyGrid>("server_map", 1, true))
//    map_(
//        GMapping::Point(map.map.info.origin.position.x,
//                        map.map.info.origin.position.y),
//        map.map.info.width * map.map.info.resolution,
//        map.map.info.height * map.map.info.resolution, map.map.info.resolution), server_node_(), robot_count_(
//        0), publish_period_(5), map_frame_("map"), map_pub_(
//        server_node_.advertise<nav_msgs::OccupancyGrid>("server_map", 1, true))
{
  // init map
  ROS_DEBUG_STREAM_NAMED("load_map", "size x: " << map_.getMapSizeX());
  ROS_DEBUG_STREAM_NAMED("load_map", "size y: " << map_.getMapSizeY());
  ROS_DEBUG_STREAM_NAMED("load_map", "size msg x: " << map.map.info.width);
  ROS_DEBUG_STREAM_NAMED("load_map", "size msg y: " << map.map.info.height);
  ROS_DEBUG_STREAM_NAMED("load_map", "size data: " << map.map.data.size());
  for (auto x = 0; x < map_.getMapSizeX(); x++)
  {
//    ROS_DEBUG_STREAM_NAMED("load_map", "x: " << x);
    for (auto y = 0; y < map_.getMapSizeY(); y++)
    {
//      ROS_DEBUG_STREAM_NAMED("load_map", "y: " << y);
      int occ =
          static_cast<int>(map.map.data[MAP_IDX(map_.getMapSizeX(), x, y)]);
      map_.cell(x, y).init(occ, map_.map2world(x,y));
//      ROS_DEBUG_STREAM_NAMED("load_map", "occ: " << occ);
    }
  }
  ROS_INFO_STREAM("Map loaded");

  // init map msg
  map_msg_.map.info.resolution = map_.getDelta();
  map_msg_.map.info.origin.position.x = map.map.info.origin.position.x;
  map_msg_.map.info.origin.position.y = map.map.info.origin.position.y;
  map_msg_.map.info.origin.position.z = 0.0;
  map_msg_.map.info.origin.orientation.x = 0.0;
  map_msg_.map.info.origin.orientation.y = 0.0;
  map_msg_.map.info.origin.orientation.z = 0.0;
  map_msg_.map.info.origin.orientation.w = 1.0;
  map_msg_.map.info.width = map_.getMapSizeX();
  map_msg_.map.info.height = map_.getMapSizeY();
  map_msg_.map.data.resize(map_msg_.map.info.width * map_msg_.map.info.height);

  ROS_INFO_STREAM(
      "delta; " << map_.getDelta() << " x; " << map_msg_.map.info.origin.position.x << " y; " << map_msg_.map.info.origin.position.y << " X; " << map_msg_.map.info.width << " Y; " << map_msg_.map.info.height);

  // init communication
  map_publish_thread_ = std::thread(&SlamServer::publishMapLoop, this);
  robot_turn_off_ = server_node_.advertiseService("robot_turn_off",
                                                  &SlamServer::robotTurnOff,
                                                  this);
  map_down_ = server_node_.advertiseService("map_download",
                                            &SlamServer::mapDownload, this);
  register_robot_ = server_node_.advertiseService("request_pose",
                                                  &SlamServer::poseRequest,
                                                  this);
  init_robot_ = server_node_.advertiseService("init_robot",
                                              &SlamServer::initRobot, this);
  pose_sub_ = server_node_.subscribe("pose", 20, &SlamServer::registerPose,
                                     this);

}

SlamServer::SlamServer() :
    map_(GMapping::Point(0, 0), -20, -20, 20, 20, 0.05), server_node_(), robot_count_(
        0), publish_period_(5), map_frame_("map"), map_pub_(
        server_node_.advertise<nav_msgs::OccupancyGrid>("server_map", 1, true))
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
  robot_turn_off_ = server_node_.advertiseService("robot_turn_off",
                                                  &SlamServer::robotTurnOff,
                                                  this);
  map_down_ = server_node_.advertiseService("map_download",
                                            &SlamServer::mapDownload, this);
  register_robot_ = server_node_.advertiseService("request_pose",
                                                  &SlamServer::poseRequest,
                                                  this);
  init_robot_ = server_node_.advertiseService("init_robot",
                                              &SlamServer::initRobot, this);
  pose_sub_ = server_node_.subscribe("pose", 20, &SlamServer::registerPose,
                                     this);

}

SlamServer::~SlamServer()
{
  map_publish_thread_.join();
}

void SlamServer::registerRobot(const std::string name)
{
  auto rob = std::make_shared<RobotHandle>(10, server_node_, name);
  rob->setServer(shared_from_this());
  rob_map_.insert(make_pair(name, rob));
  robot_count_++;
}

void SlamServer::updateMapSegment(
    server_slam::requestSeenCells::Response& response)
{
  //This is just temporary
  if (static_cast<bool>(response.b.size()))
  {
    buffer_ = response;
  }

  ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock MAP");
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_DEBUG_STREAM_NAMED("mutex", "Locked MAP");

  for (int i = 0; i < response.b.size(); i++)
  {
    map_.cell(GMapping::IntPoint(response.b[i].point.x, response.b[i].point.y)).setfromMsg(
        response.b[i].pa);
  }
  ROS_DEBUG_STREAM_NAMED("mutex", "Release MAP");
}

void SlamServer::publishMapLoop()
{
  while (ros::ok())
  {
    this->publishMap();
    ros::Duration(publish_period_).sleep();
  }
}

void SlamServer::publishMap()
{
  ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock MAP");
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_DEBUG_STREAM_NAMED("mutex", "Locked MAP");

  for (int x = 0; x < map_.getMapSizeX(); x++)
  {
    for (int y = 0; y < map_.getMapSizeY(); y++)
    {
      double occ = (double)map_.cell(GMapping::IntPoint(x, y));
//      ROS_DEBUG_STREAM_NAMED("load_map", "occ: " << occ);

      if (occ < 0)
      {
        map_msg_.map.data[MAP_IDX(map_msg_.map.info.width, x, y)] = -1;
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
  ROS_INFO_STREAM("Map was published");
  ROS_DEBUG_STREAM_NAMED("mutex", "Release MAP");
}

bool SlamServer::mapDownload(server_slam::requestSeenCells::Request & req,
                             server_slam::requestSeenCells::Response& res)
{
  ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock MAP");
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_DEBUG_STREAM_NAMED("mutex", "Locked MAP");

  if (req.a == 1)
  {
    res = buffer_;
    ROS_DEBUG_STREAM_NAMED("mutex", "Release MAP");
    return true;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("mutex", "Release MAP");
    return false;
  }
}

bool SlamServer::robotTurnOff(server_slam::turnOff::Request & req,
                              server_slam::turnOff::Response& res)
{
  ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock ROBOT");
  std::lock_guard<std::mutex> guard(saved_robots_mutex_);
  ROS_DEBUG_STREAM_NAMED("mutex", "Locked ROBOT");

  std::cerr << req.name << std::endl;
  // std::cerr << req.name.size() << std::endl;
  if (req.name.size() >= 0)
  {
    auto success = saved_robots_.insert(
        std::make_pair(
            req.name,
            GMapping::OrientedPoint(req.pose.x, req.pose.y, req.pose.theta)));
    if (!success.second)
    {
      success.first->second = GMapping::OrientedPoint(req.pose.x, req.pose.y,
                                                      req.pose.theta);
    }
    std::cerr << "x: " << req.pose.x << " y: " << req.pose.y << " yaw: "
        << req.pose.theta << std::endl;
    ROS_DEBUG_STREAM_NAMED("mutex", "Release RBOOT");
    return true;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("mutex", "Release ROBOT");
    return false;
  }
}

bool SlamServer::poseRequest(server_slam::poseRequest::Request & req,
                             server_slam::poseRequest::Response& res)
{
  ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock ROBOT");
  std::lock_guard<std::mutex> guard(saved_robots_mutex_);
  ROS_DEBUG_STREAM_NAMED("mutex", "Locked ROBOT");

  auto it = saved_robots_.find(req.name);
  if (it == saved_robots_.end())
  {
    registerRobot(req.name);
    ROS_DEBUG_STREAM_NAMED("mutex", "Release RBOOT");
    return false;
  }
  GMapping::OrientedPoint& temp = it->second;
  res.pose.x = temp.x;
  res.pose.y = temp.y;
  res.pose.theta = temp.theta;
  std::cerr << "x: " << res.pose.x << " y: " << res.pose.y << " yaw: "
      << res.pose.theta << std::endl;
  ROS_DEBUG_STREAM_NAMED("mutex", "Release RBOOT");
  return true;
}

bool SlamServer::initRobot(server_slam::initRobot::Request & req,
                           server_slam::initRobot::Response& res)
{
  {
    ROS_DEBUG_STREAM_NAMED("init_robot", "  Try to get map");
    // map

    ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock MAP");
    std::lock_guard<std::mutex> guard(map_mutex_);
    ROS_DEBUG_STREAM_NAMED("mutex", "Locked MAP");

    res.msg2d.max.x = map_.getMapSizeX();
    res.msg2d.max.y = map_.getMapSizeY();

    std::vector<server_slam::PaArray> x_dim(map_.getMapSizeX());
    for (auto x = 0; x < map_.getMapSizeX(); x++)
    {
      std::vector<server_slam::PointAccumulator> y_dim(map_.getMapSizeY());
      for (auto y = 0; y < map_.getMapSizeY(); y++)
      {
        map_.cell(x, y).setMsg(y_dim[y]);
      }
      x_dim[x].paArray = y_dim;
    }
    res.msg2d.pa2dArray = x_dim;

    res.origin.x = map_.getCenter().x;
    res.origin.y = map_.getCenter().y;

    ROS_DEBUG_STREAM_NAMED(
        "init_robot",
        "x: " << map_.getCenter().x << "y: " << map_.getCenter().y);

    res.delta = map_.getResolution();
    ROS_DEBUG_STREAM_NAMED("init_robot", "send Delta: " << res.delta);

    ROS_DEBUG_STREAM_NAMED("mutex", "Release MAP");

  }

  {
    ROS_DEBUG_STREAM_NAMED("init_robot", "Try to init the bot");

    ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock ROBOT");
    std::lock_guard<std::mutex> guard(saved_robots_mutex_);
    ROS_DEBUG_STREAM_NAMED("mutex", "Locked ROBOT");

    auto it = saved_robots_.find(req.name);
    if (it == saved_robots_.end())
    {
      ROS_DEBUG_STREAM_NAMED("init_robot", "  Init failed");
      registerRobot(req.name);
      ROS_INFO_STREAM("New robot registered: " << req.name);
      ROS_DEBUG_STREAM_NAMED("mutex", "Release RBOOT");
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Initialize robot: " << req.name);
      ROS_DEBUG_STREAM_NAMED("init_robot", "  Try to get Pose");
      // pose
      GMapping::OrientedPoint& temp = it->second;
      res.pose.x = temp.x;
      res.pose.y = temp.y;
      res.pose.theta = temp.theta;
      ROS_DEBUG_STREAM_NAMED("mutex", "Release RBOOT");
      return false;
    }
  }
}

void SlamServer::registerPose(
    const server_slam::PoseNamedStamped::ConstPtr& msg)
{
  ROS_DEBUG_STREAM_NAMED("mutex", "Going to lock ROBOT");
  std::lock_guard<std::mutex> guard(saved_robots_mutex_);
  ROS_DEBUG_STREAM_NAMED("mutex", "Locked ROBOT");

  if (msg->name.size() >= 0)
  {
//    ROS_DEBUG_STREAM(msg->name << " x: " << msg->pose.x << " y: " << msg->pose.y << " yaw: " << msg->pose.theta);
    auto it = saved_robots_.find(msg->name);
    if (it == saved_robots_.end())
    {
      saved_robots_.insert(
          {msg->name, GMapping::OrientedPoint(msg->pose.x, msg->pose.y,
                                              msg->pose.theta)});
    }
    else
    {
      it->second = GMapping::OrientedPoint(msg->pose.x, msg->pose.y,
                                           msg->pose.theta);
    }
  }
  else
  {
    ROS_ERROR_STREAM("No name transmitted");
  }
  ROS_DEBUG_STREAM_NAMED("mutex", "Release RBOOT");
}
