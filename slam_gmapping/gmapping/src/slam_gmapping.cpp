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
/* Modified by: Charles DuHadway */

/**

 @mainpage slam_gmapping

 @htmlinclude manifest.html

 @b slam_gmapping is a wrapper around the GMapping SLAM library. It reads
 laser
 scans and odometry and computes a map. This map can be
 written to a file using e.g.

 "rosrun map_server map_saver static_map:=dynamic_map"

 <hr>

 @section topic ROS topics

 Subscribes to (name/type):
 - @b "scan"/<a
 href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a>
 : data from a laser range scanner
 - @b "/tf": odometry from the robot


 Publishes to (name/type):
 - @b "/tf"/tf/tfMessage: position relative to the map


 @section services
 - @b "~dynamic_map" : returns the map


 @section parameters ROS parameters

 Reads the following parameters from the parameter server

 Parameters used by our GMapping wrapper:

 - @b "~throttle_scans": @b [int] throw away every nth laser scan
 - @b "~base_frame": @b [string] the tf frame_id to use for the robot base
 pose
 - @b "~map_frame": @b [string] the tf frame_id where the robot pose on the
 map is published
 - @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
 - @b "~map_update_interval": @b [double] time in seconds between two
 recalculations of the map


 Parameters used by GMapping itself:

 Laser Parameters:
 - @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond
 this range get discarded completely. (default: maximum laser range minus 1
 cm, as received in the the first LaserScan message)
 - @b "~/maxUrange" @b [double] maximum range of the laser scanner that is
 used for map building (default: same as maxRange)
 - @b "~/sigma" @b [double] standard deviation for the scan matching process
 (cell)
 - @b "~/kernelSize" @b [double] search window for the scan matching process
 - @b "~/lstep" @b [double] initial search step for scan matching (linear)
 - @b "~/astep" @b [double] initial search step for scan matching (angular)
 - @b "~/iterations" @b [double] number of refinement steps in the scan
 matching. The final "precision" for the match is lstep*2^(-iterations) or
 astep*2^(-iterations), respectively.
 - @b "~/lsigma" @b [double] standard deviation for the scan matching process
 (single laser beam)
 - @b "~/ogain" @b [double] gain for smoothing the likelihood
 - @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a
 match (0 = take all rays)
 - @b "~/minimumScore" @b [double] minimum score for considering the outcome
 of the scanmatching good. Can avoid 'jumping' pose estimates in large open
 spaces when using laser scanners with limited range (e.g. 5m). (0 =
 default. Scores go up to 600+, try 50 for example when experiencing
 'jumping' estimate issues)

 Motion Model Parameters (all standard deviations of a gaussian noise model)
 - @b "~/srr" @b [double] linear noise component (x and y)
 - @b "~/stt" @b [double] angular noise component (theta)
 - @b "~/srt" @b [double] linear -> angular noise component
 - @b "~/str" @b [double] angular -> linear noise component

 Others:
 - @b "~/linearUpdate" @b [double] the robot only processes new measurements
 if the robot has moved at least this many meters
 - @b "~/angularUpdate" @b [double] the robot only processes new measurements
 if the robot has turned at least this many rads

 - @b "~/resampleThreshold" @b [double] threshold at which the particles get
 resampled. Higher means more frequent resampling.
 - @b "~/particles" @b [int] (fixed) number of particles. Each particle
 represents a possible trajectory that the robot has traveled

 Likelihood sampling (used in scan matching)
 - @b "~/llsamplerange" @b [double] linear range
 - @b "~/lasamplerange" @b [double] linear step size
 - @b "~/llsamplestep" @b [double] linear range
 - @b "~/lasamplestep" @b [double] angular step size

 Initial map dimensions and resolution:
 - @b "~/xmin" @b [double] minimum x position in the map [m]
 - @b "~/ymin" @b [double] minimum y position in the map [m]
 - @b "~/xmax" @b [double] maximum x position in the map [m]
 - @b "~/ymax" @b [double] maximum y position in the map [m]
 - @b "~/delta" @b [double] size of one pixel [m]

 */

#include "slam_gmapping.h"

#include <iostream>

#include <time.h>
#include <cfloat>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping() :
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Point(0, 0, 0))), laser_count_(0), private_nh_(
        "~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer) :
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Point(0, 0, 0))), laser_count_(0), private_nh_(
        "~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL), seed_(seed), tf_(
        ros::Duration(max_duration_buffer))
{
  init();
}

void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  // gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_laser_angle_increment_ = 0.0;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;

  // Parameters used by our GMapping wrapper
  if (!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;

  if (!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";

  if (!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";

  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;

  if (!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);

  // Parameters used by GMapping itself
  maxUrange_ = 0.0;
  maxRange_ = 0.0; // preliminary default, will be set in initMapper()

  if (!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;

  if (!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;

  if (!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;

  if (!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;

  if (!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;

  if (!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;

  if (!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;

  if (!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;

  if (!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;

  if (!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;

  if (!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;

  if (!private_nh_.getParam("str", str_))
    str_ = 0.1;

  if (!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;

  if (!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;

  if (!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;

  if (!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;

  if (!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;

  if (!private_nh_.getParam("particles", particles_))
    particles_ = 30;

  if (!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;

  if (!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;

  if (!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;

  if (!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;

  if (!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;

  if (!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.5;

  if (!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;

  if (!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;

  if (!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;

  if (!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;

  if (!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
}

void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  seen_cells_ = node_.advertise<nav_msgs::OccupancyGrid>("seen_cells_map", 1, true);
  active_cells_ = node_.advertise<nav_msgs::OccupancyGrid>("active_cells_map", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("ref_map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  particlePCL_ = node_.advertise<sensor_msgs::PointCloud>("particles", 1, true);

  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  mapUp_ = node_.advertiseService("map_upstream", &SlamGMapping::mapUpstream, this);

  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));

  download_cl_ = node_.serviceClient<server_slam::requestSeenCells>("map_download");
  download_thread_ = std::thread(&SlamGMapping::downloadLoop, this);
}

void SlamGMapping::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);

  while (ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if (transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;

  if (gsp_laser_)
    delete gsp_laser_;

  if (gsp_odom_)
    delete gsp_odom_;

  if (scan_filter_)
    delete scan_filter_;

  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the laser's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0)), t,
                              laser_frame_);
  tf::Stamped<tf::Transform> odom_pose;

  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch (tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw);
  return true;
}

bool SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;

  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch (tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
    return false;
  }
  // Check that laserscan is from -x to x in angles:
  if (fabs(fabs(scan.angle_min) - fabs(scan.angle_max)) > FLT_EPSILON)
  {
    ROS_ERROR("Scan message must contain angles from -x to x, i.e. angle_min = -angle_max");
    return false;
  }

  // create a point 1m above the laser position and transform it into the
  // laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp, base_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s", e.what());
    return false;
  }
  // gmapping doesnt take roll or pitch into account. So check for correct
  // sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  int orientationFactor;

  if (up.z() > 0)
  {
    orientationFactor = 1;
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    orientationFactor = -1;
    ROS_INFO("Laser is mounted upside down.");
  }

  angle_min_ = orientationFactor * scan.angle_min;
  angle_max_ = orientationFactor * scan.angle_max;
  gsp_laser_angle_increment_ = orientationFactor * scan.angle_increment;
  ROS_DEBUG("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f", angle_min_, angle_max_,
            gsp_laser_angle_increment_);

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");

  if (!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;

  if (!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(gsp_laser_angle_increment_), gmap_pose,
                                         0.0, maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);

  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;

  if (!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_, lstep_, astep_, iterations_, lsigma_, ogain_,
                              lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(true);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);

  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1, seed_);

  ROS_INFO("Initialization complete");

  return true;
}

bool SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if (!getOdomPose(gmap_pose, scan.header.stamp))
    return false;

  if (scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double *ranges_double = new double[scan.ranges.size()];

  // If the angle increment is negative, we have to invert the order of the
  // readings.
  if (gsp_laser_angle_increment_ < 0)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();

    for (int i = 0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if (scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  }
  else
  {
    for (unsigned int i = 0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if (scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(), ranges_double, gsp_laser_, scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
   ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
   scan.header.stamp.toSec(),
   gmap_pose.x,
   gmap_pose.y,
   gmap_pose.theta);
   */

  return gsp_->processScan(reading);
}

void SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;

  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0, 0);

  // We can't initialize the mapper until we've got the first scan
  if (!got_first_scan_)
  {
    if (!initMapper(*scan))
      return;

    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  gsp_mutex_.lock();

  bool b = addScan(*scan, odom_pose);

  gsp_mutex_.unlock();

  if (b)
  {
    ROS_DEBUG("scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y,
              mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta),
                                               tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta),
                                                tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if (!got_map_ || ((scan->header.stamp - last_map_update) > map_update_interval_))
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  }
}

double SlamGMapping::computePoseEntropy()
{
  double weight_total = 0.0;

  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end(); ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;

  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end(); ++it)
  {
    if (it->weight / weight_total > 0.0)
      entropy += it->weight / weight_total * log(it->weight / weight_total);
  }
  return -entropy;
}

void SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  boost::mutex::scoped_lock map_lock(map_mutex_);

  GMapping::ScanMatcherMap& ref_map = *gsp_->m_refMap_ptr;

  if (!got_map_)
  {
    ref_map_.map.info.resolution = delta_;
    ref_map_.map.info.origin.position.x = 0.0;
    ref_map_.map.info.origin.position.y = 0.0;
    ref_map_.map.info.origin.position.z = 0.0;
    ref_map_.map.info.origin.orientation.x = 0.0;
    ref_map_.map.info.origin.orientation.y = 0.0;
    ref_map_.map.info.origin.orientation.z = 0.0;
    ref_map_.map.info.origin.orientation.w = 1.0;

    ref_map_.map.info.width = ref_map.getMapSizeX();
    ref_map_.map.info.height = ref_map.getMapSizeY();
    ref_map_.map.info.origin.position.x = xmin_;
    ref_map_.map.info.origin.position.y = ymin_;
    ref_map_.map.data.resize(ref_map_.map.info.width * ref_map_.map.info.height);

    seen_map_.map.info.resolution = delta_;
    seen_map_.map.info.origin.position.x = 0.0;
    seen_map_.map.info.origin.position.y = 0.0;
    seen_map_.map.info.origin.position.z = 0.0;
    seen_map_.map.info.origin.orientation.x = 0.0;
    seen_map_.map.info.origin.orientation.y = 0.0;
    seen_map_.map.info.origin.orientation.z = 0.0;
    seen_map_.map.info.origin.orientation.w = 1.0;

    seen_map_.map.info.width = ref_map.getMapSizeX();
    seen_map_.map.info.height = ref_map.getMapSizeY();
    seen_map_.map.info.origin.position.x = xmin_;
    seen_map_.map.info.origin.position.y = ymin_;
    seen_map_.map.data.resize(seen_map_.map.info.width * seen_map_.map.info.height);

    active_map_.map.info.resolution = delta_;
    active_map_.map.info.origin.position.x = 0.0;
    active_map_.map.info.origin.position.y = 0.0;
    active_map_.map.info.origin.position.z = 0.0;
    active_map_.map.info.origin.orientation.x = 0.0;
    active_map_.map.info.origin.orientation.y = 0.0;
    active_map_.map.info.origin.orientation.z = 0.0;
    active_map_.map.info.origin.orientation.w = 1.0;

    active_map_.map.info.width = ref_map.getMapSizeX();
    active_map_.map.info.height = ref_map.getMapSizeY();
    active_map_.map.info.origin.position.x = xmin_;
    active_map_.map.info.origin.position.y = ymin_;
    active_map_.map.data.resize(active_map_.map.info.width * active_map_.map.info.height);
  }

  GMapping::Point center;
  center.x = (xmin_ + xmax_) / 2.0;
  center.y = (ymin_ + ymax_) / 2.0;

  // the map may have expanded, so resize ros message as well
  if ((ref_map_.map.info.width != (unsigned int)ref_map.getMapSizeX())
      || (ref_map_.map.info.height != (unsigned int)ref_map.getMapSizeY()))
  {
    ROS_WARN_STREAM("This should not have happened, tried to resize the map");
  }

  const GMapping::GridSlamProcessor::Particle& best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

  for (int x = 0; x < ref_map.getMapSizeX(); x++)
  {
    for (int y = 0; y < ref_map.getMapSizeY(); y++)
    {
      GMapping::IntPoint p(x, y);

      // Set map for ref map
      double ref_occ = (double)ref_map.cell(p);

      if (ref_occ < 0)
      {
        ref_map_.map.data[MAP_IDX(ref_map_.map.info.width, x, y)] = -1;
      }
      else
      {
        ref_map_.map.data[MAP_IDX(ref_map_.map.info.width, x, y)] = ref_occ * 100;
      }

      // Set map for seen cells
      if (best.seenCells.find(p) != best.seenCells.end())
      {
        seen_map_.map.data[MAP_IDX(seen_map_.map.info.width, x, y)] = 100;
      }
      else
      {
        seen_map_.map.data[MAP_IDX(seen_map_.map.info.width, x, y)] = -1;
      }

      // Set active Cells
      auto it = best.activeCells.find(p);

      if (it == best.activeCells.end())
      {
        active_map_.map.data[MAP_IDX(active_map_.map.info.width, x, y)] = -1;
      }
      else
      {
        active_map_.map.data[MAP_IDX(active_map_.map.info.width, x, y)] = (double)*it->second * 100;
      }

    }
  }

  got_map_ = true;

  // make sure to set the header information on the map
  ref_map_.map.header.stamp = ros::Time::now();
  ref_map_.map.header.frame_id = tf_.resolve(map_frame_);
  active_map_.map.header.stamp = ros::Time::now();
  active_map_.map.header.frame_id = tf_.resolve(map_frame_);
  seen_map_.map.header.stamp = ros::Time::now();
  seen_map_.map.header.frame_id = tf_.resolve(map_frame_);

  active_cells_.publish(active_map_.map);
  seen_cells_.publish(seen_map_.map);
  sst_.publish(ref_map_.map);
  sstm_.publish(ref_map_.map.info);

  // print particles
  sensor_msgs::PointCloud particleVis_;

  particleVis_.header.frame_id = tf_.resolve(map_frame_);
  particleVis_.header.stamp = ros::Time::now();

  const GMapping::GridSlamProcessor::ParticleVector& vis_particles_ = gsp_->getParticles();

  for (GMapping::GridSlamProcessor::ParticleVector::const_iterator it = vis_particles_.begin();
      it != vis_particles_.end(); it++)
  {
    geometry_msgs::Point32 point;
    point.x = it->pose.x;
    point.y = it->pose.y;
    point.z = 0;

    particleVis_.points.push_back(point);
  }

  particlePCL_.publish(particleVis_);
}

bool SlamGMapping::mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
  boost::mutex::scoped_lock map_lock(map_mutex_);

  if (got_map_ && ref_map_.map.info.width && ref_map_.map.info.height)
  {
    res = ref_map_;
    return true;
  }
  else
    return false;
}

bool SlamGMapping::mapUpstream(server_slam::requestSeenCells::Request & req,
                               server_slam::requestSeenCells::Response& res)
{
  boost::mutex::scoped_lock map_lock(map_mutex_);

  if (req.a == 1)
  {
    const GMapping::GridSlamProcessor::Particle& best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

    std::vector<server_slam::PaCell> temp(best.seenCells.size());

    int i = 0;
    for (auto it = best.seenCells.begin(); it != best.seenCells.end(); it++, i++)
    {
      temp[i].point.x = it->x;
      temp[i].point.y = it->y;
      best.getCell(gsp_->m_refMap_ptr, GMapping::IntPoint(it->x, it->y)).setMsg(temp[i].pa);
    }
    res.b = temp;

    gsp_->clearSeenCells();

    const GMapping::GridSlamProcessor::Particle& best2 = gsp_->getParticles()[gsp_->getBestParticleIndex()];
    return true;
  }
  else
  {
    return false;
  }
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

void SlamGMapping::downloadLoop()
{
  while (ros::ok())
  {
    ros::Duration(map_update_interval_ * 5).sleep();
    ROS_INFO("I will send a request");
    this->sendDownloadRequest();
  }
}

void SlamGMapping::sendDownloadRequest()
{
  boost::mutex::scoped_lock map_lock(map_mutex_);

  gsp_mutex_.lock();

  download_srv_.request.a = 1;

  if (download_cl_.call(download_srv_))
  {
    ROS_INFO("I got a response");


    for (int i = 0; i < download_srv_.response.b.size(); i++)
    {
      GMapping::IntPoint p(download_srv_.response.b[i].point.x, download_srv_.response.b[i].point.y);

      gsp_->m_refMap_ptr->cell(p).setfromMsg(download_srv_.response.b[i].pa);

      for (auto particle_it = gsp_->getParticles().begin(); particle_it != gsp_->getParticles().end(); particle_it++)
      {
        auto it = particle_it->activeCells.find(p);
        if (it != particle_it->activeCells.end())
        {
          particle_it->activeCells.erase(p);
        }
      }

    }

  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

  gsp_mutex_.unlock();
}
