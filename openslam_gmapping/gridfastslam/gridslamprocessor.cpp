#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include <gmapping/utils/stat.h>
#include <gmapping/gridfastslam/gridslamprocessor.h>

// #define MAP_CONSISTENCY_CHECK
// #define GENERATE_TRAJECTORIES

namespace GMapping
{
const double m_distanceThresholdCheck = 20;

using namespace std;

GridSlamProcessor::GridSlamProcessor()
{
  period_ = 5.0;
  m_obsSigmaGain = 1;
  m_resampleThreshold = 0.5;
  m_minimumScore = 0.;
}

GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) :
    last_update_time_(0.0), m_particles(gsp.m_particles)
{
  period_ = 5.0;

  m_obsSigmaGain = gsp.m_obsSigmaGain;
  m_resampleThreshold = gsp.m_resampleThreshold;
  m_minimumScore = gsp.m_minimumScore;

  m_beams = gsp.m_beams;
  m_indexes = gsp.m_indexes;
  m_motionModel = gsp.m_motionModel;
  m_resampleThreshold = gsp.m_resampleThreshold;
  m_matcher = gsp.m_matcher;

  m_count = gsp.m_count;
  m_readingCount = gsp.m_readingCount;
  m_lastPartPose = gsp.m_lastPartPose;
  m_pose = gsp.m_pose;
  m_odoPose = gsp.m_odoPose;
  m_linearDistance = gsp.m_linearDistance;
  m_angularDistance = gsp.m_angularDistance;
  m_neff = gsp.m_neff;

  cerr << "FILTER COPY CONSTRUCTOR" << endl;
  cerr << "m_odoPose=" << m_odoPose.x << " " << m_odoPose.y << " " << m_odoPose.theta << endl;
  cerr << "m_lastPartPose=" << m_lastPartPose.x << " " << m_lastPartPose.y << " " << m_lastPartPose.theta << endl;
  cerr << "m_linearDistance=" << m_linearDistance << endl;
  cerr << "m_angularDistance=" << m_linearDistance << endl;

  m_xmin = gsp.m_xmin;
  m_ymin = gsp.m_ymin;
  m_xmax = gsp.m_xmax;
  m_ymax = gsp.m_ymax;
  m_delta = gsp.m_delta;

  m_regScore = gsp.m_regScore;
  m_critScore = gsp.m_critScore;
  m_maxMove = gsp.m_maxMove;

  m_linearThresholdDistance = gsp.m_linearThresholdDistance;
  m_angularThresholdDistance = gsp.m_angularThresholdDistance;
  m_obsSigmaGain = gsp.m_obsSigmaGain;

#ifdef MAP_CONSISTENCY_CHECK
  cerr << __PRETTY_FUNCTION__ << ": trajectories copy.... ";
#endif // ifdef MAP_CONSISTENCY_CHECK

  //  TNodeVector v = gsp.getTrajectories();
  //  for (unsigned int i = 0; i < v.size(); i++)
  //  {
  //    m_particles[i].node = v[i];
  //  }
#ifdef MAP_CONSISTENCY_CHECK
  cerr << "end" << endl;
#endif // ifdef MAP_CONSISTENCY_CHECK

  cerr << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ...";

  //  updateTreeWeights(false);
  normalize();
  cerr << ".done!" << endl;
}

GridSlamProcessor * GridSlamProcessor::clone() const
{

  GridSlamProcessor *cloned = new GridSlamProcessor(*this);

  return cloned;
}

void GridSlamProcessor::setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt,
                                              double aopt, int iterations, double likelihoodSigma,
                                              double likelihoodGain, unsigned int likelihoodSkip)
{
  m_obsSigmaGain = likelihoodGain;
  m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma,
                                  likelihoodSkip);

  ROS_INFO_STREAM(
      " -maxUrange " << urange << " -maxUrange " << range << " -sigma     " << sigma << " -kernelSize " << kernsize << " -lstep " << lopt << " -lobsGain " << m_obsSigmaGain << " -astep " << aopt);
}

void GridSlamProcessor::setMotionModelParameters(double srr, double srt, double str, double stt)
{
  m_motionModel.srr = srr;
  m_motionModel.srt = srt;
  m_motionModel.str = str;
  m_motionModel.stt = stt;

  ROS_INFO_STREAM(" -srr " << srr << " -srt " << srt << " -str " << str << " -stt " << stt);
}

void GridSlamProcessor::setUpdateDistances(double linear, double angular, double resampleThreshold)
{
  m_linearThresholdDistance = linear;
  m_angularThresholdDistance = angular;
  m_resampleThreshold = resampleThreshold;

  ROS_INFO_STREAM(
      " -linearUpdate " << linear << " -angularUpdate " << angular << " -resampleThreshold " << m_resampleThreshold);
}

void GridSlamProcessor::setSensorMap(const SensorMap& smap)
{
  /*
   Construct the angle table for the sensor

   FIXME For now detect the readings of only the front laser, and assume its
   pose is in the center of the robot
   */

  SensorMap::const_iterator laser_it = smap.find(std::string("FLASER"));

  if (laser_it == smap.end())
  {
    cerr << "Attempting to load the new carmen log format" << endl;
    laser_it = smap.find(std::string("ROBOTLASER1"));
    assert(laser_it != smap.end());
  }
  const RangeSensor *rangeSensor = dynamic_cast<const RangeSensor *>((laser_it->second));
  assert(rangeSensor && rangeSensor->beams().size());

  m_beams = static_cast<unsigned int>(rangeSensor->beams().size());
  double *angles = new double[rangeSensor->beams().size()];

  for (unsigned int i = 0; i < m_beams; i++)
  {
    angles[i] = rangeSensor->beams()[i].pose.theta;
  }
  m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
  delete[] angles;
}

void GridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta,
                             OrientedPoint initialPose)
{
  m_xmin = xmin;
  m_ymin = ymin;
  m_xmax = xmax;
  m_ymax = ymax;
  m_delta = delta;

  ROS_INFO_STREAM(
      " -xmin " << m_xmin << " -xmax " << m_xmax << " -ymin " << m_ymin << " -ymax " << m_ymax << " -delta " << m_delta << " -particles " << size);

  m_particles.clear();

  //  TNode* node = new TNode(initialPose, 0, 0, 0);
  m_refMap_ptr = make_shared<ScanMatcherMap>(Point(xmin + xmax, ymin + ymax) * .5, xmax - xmin, ymax - ymin, delta);

  for (unsigned int i = 0; i < size; i++)
  {
    m_particles.push_back(Particle(*m_refMap_ptr));
    m_particles.back().pose = initialPose;
    //m_particles.back().previousPose = initialPose;
    m_particles.back().setWeight(0);
    //m_particles.back().previousIndex = 0;

    // this is not needed
    //		m_particles.back().node=new TNode(initialPose, 0, node, 0);

    // we use the root directly
    //    m_particles.back().node = node;
  }
  m_neff = (double)size;
  m_count = 0;
  m_readingCount = 0;
  m_linearDistance = m_angularDistance = 0;
}

void GridSlamProcessor::processTruePos(const OdometryReading& o)
{
  const OdometrySensor *os = dynamic_cast<const OdometrySensor *>(o.getSensor());
}

bool GridSlamProcessor::processScan(const RangeReading& reading, int adaptParticles)
{
  /**retireve the position from the reading, and compute the odometry*/
  OrientedPoint relPose = reading.getPose();

  if (!m_count)
  {
    m_lastPartPose = m_odoPose = relPose;
  }

  // write the state of the reading and update all the particles using the
  // motion model
  for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
  {
    OrientedPoint& pose(it->pose);
    pose = m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);
  }

  // invoke the callback
  onOdometryUpdate();

  // accumulate the robot translation and rotation
  OrientedPoint move = relPose - m_odoPose;
  move.theta = atan2(sin(move.theta), cos(move.theta));
  m_linearDistance += sqrt(move * move);
  m_angularDistance += fabs(move.theta);

  // if the robot jumps throw a warning
  if (m_linearDistance > m_distanceThresholdCheck)
  {
    cerr << "***********************************************************************" << endl;
    cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
    cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
    cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y << " " << m_odoPose.theta << endl;
    cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y << " " << relPose.theta
        << endl;
    cerr << "***********************************************************************" << endl;
    cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
    cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
    cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
    cerr << "***********************************************************************" << endl;
  }

  m_odoPose = relPose;

  bool processed = false;

  // process a scan only if the robot has traveled a given distance or a certain
  // amount of time has elapsed
  if (!m_count || (m_linearDistance >= m_linearThresholdDistance) || (m_angularDistance >= m_angularThresholdDistance)
      || ((period_ >= 0.0) && ((reading.getTime() - last_update_time_) > period_)))
  {
    last_update_time_ = reading.getTime();

    ROS_INFO_STREAM(
        "update frame " << m_readingCount << endl << "update ld=" << m_linearDistance << " ad=" << m_angularDistance);

    ROS_ERROR_STREAM(
        "Laser Pose= " << reading.getPose().x << " " << reading.getPose().y << " " << reading.getPose().theta);

    // this is for converting the reading in a scan-matcher feedable form
    assert(reading.size() == m_beams);
    double *plainReading = new double[m_beams];

    for (unsigned int i = 0; i < m_beams; i++)
    {
      plainReading[i] = reading[i];
    }
    ROS_INFO_STREAM("m_count " << m_count);

    RangeReading *reading_copy = new RangeReading(reading.size(), &(reading[0]),
                                                  static_cast<const RangeSensor *>(reading.getSensor()),
                                                  reading.getTime());

    if (m_count > 0)
    {
      scanMatch(plainReading);

      onScanmatchUpdate();

      normalize();

      //      updateTreeWeights(false);

      ROS_INFO_STREAM("neff= " << m_neff);

      resample(plainReading, adaptParticles, reading_copy);
    }
    else
    {
      ROS_INFO_STREAM("Registering First Scan");

      for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
      {
        // m_matcher.invalidateActiveArea();
        // m_matcher.computeActiveArea(it->map, it->pose, plainReading);
        // m_matcher.registerScan(it->map, it->pose, plainReading);
        //        m_matcher.registerActiveCells(it->activeCells, *m_refMap_ptr,
        // it->pose, plainReading);
        //        m_matcher.registerSeenCells(it->seenCells, *m_refMap_ptr,
        // it->pose, plainReading);
        m_matcher.registerCells(it->seenCells, it->activeCells, *m_refMap_ptr, it->pose, plainReading);

        // cyr: not needed anymore, particles refer to the root in the
        // beginning!
        //        TNode* node = new TNode(it->pose, 0., it->node, 0);
        //        //node->reading=0;
        //        node->reading = reading_copy;
        //        it->node = node;
      }
    }

    //		cerr  << "Tree: normalizing, resetting and propagating weights
    // at the end..." ;
    normalize();

    //    updateTreeWeights(false);
    //		cerr  << ".done!" <<endl;

    delete[] plainReading;
    m_lastPartPose = m_odoPose; // update the past pose for the next
                                // iteration
    m_linearDistance = 0;
    m_angularDistance = 0;
    m_count++;
    processed = true;

  }

  m_readingCount++;
  return processed;
}

int GridSlamProcessor::getBestParticleIndex() const
{
  unsigned int bi = 0;
  double bw = -std::numeric_limits<double>::max();

  for (unsigned int i = 0; i < m_particles.size(); i++)
    if (bw < m_particles[i].weightSum)
    {
      bw = m_particles[i].weightSum;
      bi = i;
    }
  return (int)bi;
}

void GridSlamProcessor::clearSeenCells()
{
  for (auto it = m_particles.begin(); it != m_particles.end(); it++)
  {
    it->seenCells.clear();
  }
}

void GridSlamProcessor::onScanmatchUpdate()
{
}

void GridSlamProcessor::onResampleUpdate()
{
  for (int x = 0; x < m_refMap_ptr->getMapSizeX(); x++)
  {
    for (int y = 0; y < m_refMap_ptr->getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      IntPoint p(x, y);

      // double occ = best_map.cell(p);

      m_refMap_ptr->cell(p).updateNew(no);
    }
  }
}

void GridSlamProcessor::onOdometryUpdate()
{
}
}

// end namespace
