/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"

#include "server_slam/requestSeenCells.h"
#include "server_slam/PointAccumulator.h"
#include "server_slam/turnOff.h"
#include "server_slam/poseRequest.h"
#include "server_slam/PoseNamedStamped.h"
#include "server_slam/initRobot.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>
#include <thread>
#include <string>

class SlamGMapping
{
public:

  SlamGMapping();
  SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer);
  ~SlamGMapping();

  void init();
  void startLiveSlam();
  void startReplay(const std::string& bag_fname, std::string scan_topic);
  void publishTransform();
  void publishPose();

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool mapCallback(nav_msgs::GetMap::Request & req, nav_msgs::GetMap::Response& res);
  bool mapUpstream(server_slam::requestSeenCells::Request & req, server_slam::requestSeenCells::Response& res);
  void publishLoop(double transform_publish_period);
  void downloadLoop();
  void sendDownloadRequest();

private:

  std::string name;

  ros::NodeHandle node_;
  ros::NodeHandle private_nh_;

  ros::Publisher entropy_publisher_;
  ros::Publisher active_cells_;
  ros::Publisher seen_cells_;
  ros::Publisher particlePCL_;
  ros::Publisher sst_;
  ros::Publisher sstm_;
  ros::Publisher pose_publisher_;
  ros::ServiceServer ss_;
  ros::ServiceServer mapUp_;

  ros::ServiceClient download_cl_;
  ros::ServiceClient turn_off_;
  server_slam::requestSeenCells download_srv_;
  std::thread download_thread_;

  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *scan_filter_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> *scan_filter_;
  tf::TransformBroadcaster *tfB_;

  GMapping::GridSlamProcessor *gsp_;
  GMapping::RangeSensor *gsp_laser_;
  double gsp_laser_angle_increment_;
  double angle_min_;
  double angle_max_;
  unsigned int gsp_laser_beam_count_;
  GMapping::OdometrySensor *gsp_odom_;

  bool got_first_scan_;
  bool got_map_;

  nav_msgs::GetMap::Response ref_map_;
  nav_msgs::GetMap::Response seen_map_;
  nav_msgs::GetMap::Response active_map_;

  server_slam::PoseNamedStamped pose_msg_;

  ros::Duration map_update_interval_;
  tf::Transform map_to_odom_;
  boost::mutex map_to_odom_mutex_;
  boost::mutex map_mutex_;
  boost::mutex gsp_mutex_;

  int laser_count_;
  int throttle_scans_;

  boost::thread *transform_thread_;

  std::string base_frame_;
  std::string laser_frame_;
  std::string map_frame_;
  std::string odom_frame_;

  void updateMap(const sensor_msgs::LaserScan& scan);
  bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time & t);
  bool initMapper(const sensor_msgs::LaserScan& scan);
  bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint & gmap_pose);
  double computePoseEntropy();
  void setInitialMap();
  void sendInitRequest();

  // Parameters used by GMapping
  double maxRange_;
  double maxUrange_;
  double maxrange_;
  double minimum_score_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;
  double srr_;
  double srt_;
  double str_;
  double stt_;
  double linearUpdate_;
  double angularUpdate_;
  double temporalUpdate_;
  double resampleThreshold_;
  int particles_;
//  double xmin_;
//  double ymin_;
//  double xmax_;
//  double ymax_;
  double xcenter_;
  double ycenter_;
  double delta_;
  double occ_thresh_;
  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;

  unsigned long int seed_;

  double transform_publish_period_;
  double tf_delay_;
};
