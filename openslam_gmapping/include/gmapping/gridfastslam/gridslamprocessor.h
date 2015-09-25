#ifndef GRIDSLAMPROCESSOR_H
#define GRIDSLAMPROCESSOR_H

#include <ros/ros.h>
#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <gmapping/particlefilter/particlefilter.h>
#include <gmapping/particlefilter/Particle.h>
#include <gmapping/utils/point.h>
#include <gmapping/utils/macro_params.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/scanmatcher/smmap.h>
#include <gmapping/sensor/sensor_base/sensorreading.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_odometry/odometryreading.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include "motionmodel.h"
#include <memory>
#include <boost/multi_array.hpp>

namespace GMapping
{
/**This class defines the basic GridFastSLAM algorithm.  It
 implements a rao blackwellized particle filter. Each particle
 has its own map and robot pose.<br> This implementation works
 as follows: each time a new pair odometry/laser reading is
 received, the particle's robot pose is updated according to the
 motion model.  This pose is subsequently used for initalizing a
 scan matching algorithm.  The scanmatcher performs a local
 optimization for each particle.  It is initialized with the
 pose drawn from the motion model, and the pose is corrected
 according to the each particle map.<br>
 In order to avoid unnecessary computation the filter state is updated
 only when the robot moves more than a given threshold.
 */
/**This class defines a particle of the filter. Each particle has a map, a
 pose, a weight and retains the current node in the trajectory tree*/

class GridSlamProcessor
{
public:

  /** Constructs a GridSlamProcessor, initialized with the default parameters
   */
  GridSlamProcessor();

  /** @returns  a deep copy of the grid slam processor with all the internal
   structures.
   */
  GridSlamProcessor* clone() const;

  /**Deleted the gridslamprocessor*/
  //virtual ~GridSlamProcessor();
  // methods for accessing the parameters
  void setSensorMap(const SensorMap& smap);
//  void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta,
//            OrientedPoint initialPose = OrientedPoint(0, 0, 0));

  void init(unsigned int size, const Point& center, int xmax, int ymax, double delta, const OrientedPoint& initialPose =
                OrientedPoint(0, 0, 0));
  void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
                             int iterations, double likelihoodSigma = 1, double likelihoodGain = 1,
                             unsigned int likelihoodSkip = 0);
  void setMotionModelParameters(double srr, double srt, double str, double stt);
  void setUpdateDistances(double linear, double angular, double resampleThreshold);
  void setUpdatePeriod(double p)
  {
    period_ = p;
  }

  // the "core" algorithm
  void processTruePos(const OdometryReading& odometry);
  bool processScan(const RangeReading& reading, int adaptParticles = 0);

  /**This method copies the state of the filter in a tree.
   The tree is represented through reversed pointers (each node has a
   pointer to its parent).
   The leafs are stored in a vector, whose size is the same as the number of
   particles.
   @returns the leafs of the tree
   */
//    TNodeVector getTrajectories() const;
//    void        integrateScanSequence(TNode *node);
  /**the scanmatcher algorithm*/
  ScanMatcher m_matcher;

  /**@returns the particles*/
  inline const ParticleVector& getParticles() const
  {
    return m_particles;
  }

  inline ParticleVector& getParticles()
  {
    return m_particles;
  }

  inline const std::vector<unsigned int>& getIndexes() const
  {
    return m_indexes;
  }
  int getBestParticleIndex() const;

  void clearSeenCells();

  // callbacks
  virtual void onOdometryUpdate();
  virtual void onResampleUpdate();
  virtual void onScanmatchUpdate();

  // accessor methods

  /**the maxrange of the laser to consider */
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    laserMaxRange,
  protected,
  public,
  public)
  ;

  /**the maximum usable range of the laser. A beam is cropped to this value.
   [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher, double, usableRange, protected, public,
  public)
  ;

  /**The sigma used by the greedy endpoint matching. [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    gaussianSigma,
  protected,
  public,
  public)
  ;

  /**The sigma  of a beam used for likelihood computation [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    likelihoodSigma,
  protected,
  public,
  public)
  ;

  /**The kernel in which to look for a correspondence[scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher, int, kernelSize, protected, public, public)
  ;

  /**The optimization step in rotation [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    optAngularDelta,
  protected,
  public,
  public)
  ;

  /**The optimization step in translation [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    optLinearDelta,
  protected,
  public,
  public)
  ;

  /**The number of iterations of the scanmatcher [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    unsigned int,
    optRecursiveIterations,
  protected,
  public,
  public)
  ;

  /**the beams to skip for computing the likelihood (consider a beam every
   likelihoodSkip) [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    unsigned int,
    likelihoodSkip,
  protected,
  public,
  public)
  ;

  /**translational sampling range for the likelihood [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    llsamplerange,
  protected,
  public,
  public)
  ;

  /**angular sampling range for the likelihood [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    lasamplerange,
  protected,
  public,
  public)
  ;

  /**translational sampling range for the likelihood [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    llsamplestep,
  protected,
  public,
  public)
  ;

  /**angular sampling step for the likelihood [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    double,
    lasamplestep,
  protected,
  public,
  public)
  ;

  /**enlarge the map when the robot goes out of the boundaries [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher, bool, enlargeStep, protected, public, public)
  ;

  /**pose of the laser wrt the robot [scanmatcher]*/
MEMBER_PARAM_SET_GET(m_matcher,
    OrientedPoint,
    laserPose,
  protected,
  public,
  public)
  ;

  /**odometry error in translation as a function of translation (rho/rho)
   [motionmodel]*/
STRUCT_PARAM_SET_GET(m_motionModel, double, srr, protected, public, public)
  ;

  /**odometry error in translation as a function of rotation (rho/theta)
   [motionmodel]*/
STRUCT_PARAM_SET_GET(m_motionModel, double, srt, protected, public, public)
  ;

  /**odometry error in rotation as a function of translation (theta/rho)
   [motionmodel]*/
STRUCT_PARAM_SET_GET(m_motionModel, double, str, protected, public, public)
  ;

  /**odometry error in  rotation as a function of rotation (theta/theta)
   [motionmodel]*/
STRUCT_PARAM_SET_GET(m_motionModel, double, stt, protected, public, public)
  ;

  /**minimum score for considering the outcome of the scanmatching good*/
PARAM_SET_GET(double, minimumScore, protected, public, public)
  ;

protected:

  /**Copy constructor*/
  GridSlamProcessor(const GridSlamProcessor &gsp);

  /**the laser beams*/
  unsigned int m_beams;
  double last_update_time_;
  double period_;

  /**the particles*/
  ParticleVector m_particles;

  /**the particle indexes after resampling (internally used)*/
  std::vector<unsigned int> m_indexes;

  /**the particle weights (internally used)*/
  std::vector<double> m_weights;

  /**the motion model*/
  MotionModel m_motionModel;

  /**this sets the neff based resampling threshold*/
PARAM_SET_GET(double, resampleThreshold, protected, public, public)
  ;

  // state
  int m_count, m_readingCount;
  OrientedPoint m_lastPartPose;
  OrientedPoint m_odoPose;
  OrientedPoint m_pose;
  double m_linearDistance, m_angularDistance;PARAM_GET(double,
    neff,
  protected,
  public)
  ;


  // processing parameters (resolution of the map)
PARAM_GET(double, delta, protected, public)
  ;

  // registration score (if a scan score is above this threshold it is
  // registered in the map)
PARAM_SET_GET(double, regScore, protected, public, public)
  ;

  // registration score (if a scan score is below this threshold a scan
  // matching failure is reported)
PARAM_SET_GET(double, critScore, protected, public, public)
  ;

  // registration score maximum move allowed between consecutive scans
PARAM_SET_GET(double, maxMove, protected, public, public)
  ;

  // process a scan each time the robot translates of linearThresholdDistance
PARAM_SET_GET(double, linearThresholdDistance, protected, public, public)
  ;

  // process a scan each time the robot rotates more than
  // angularThresholdDistance
PARAM_SET_GET(double, angularThresholdDistance, protected, public, public)
  ;

  // smoothing factor for the likelihood
PARAM_SET_GET(double, obsSigmaGain, protected, public, public)
  ;

  // HMM Extension
  std::shared_ptr<ScanMatcherMap> m_refMap_ptr;

  // the functions below performs side effect on the internal structure,
  // should be called only inside the processScan method

private:

  /**scanmatches all the particles*/
  inline void scanMatch(const double *plainReading);

  /**normalizes the particle weights*/
  inline void normalize();

  // return if a resampling occured or not
  inline bool resample(const double *plainReading, int adaptParticles, const RangeReading *rr = 0);

  inline void updateRefMap();

};

/**Just scan match every single particle.
 If the scan matching fails, the particle gets a default likelihood.*/
inline void GridSlamProcessor::scanMatch(const double *plainReading)
{
  // sample a new pose from each scan in the reference

  double sumScore = 0;

//    std::cerr << "here it starts" << std::endl;
  for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
  {
    OrientedPoint corrected;
    double score, l, s;
    score = m_matcher.optimize(corrected, *m_refMap_ptr, (*it)->pose, plainReading, (*it)->p_map_);

    //    it->pose=corrected;
    if (score > m_minimumScore)
    {
      (*it)->pose = corrected;
    }
    else
    {
      ROS_INFO_STREAM("Scan Matching Failed, using odometry. Likelihood=" << l);
      ROS_INFO_STREAM("lp:" << m_lastPartPose.x << " " << m_lastPartPose.y << " " << m_lastPartPose.theta);
      ROS_INFO_STREAM("op:" << m_odoPose.x << " " << m_odoPose.y << " " << m_odoPose.theta);
    }

    m_matcher.likelihoodAndScore(s, l, *m_refMap_ptr, (*it)->pose, plainReading, (*it)->p_map_);
    sumScore += score;
    (*it)->weight += l;
    (*it)->weightSum += l;

  }

  ROS_INFO_STREAM("Average Scan Matching Score=" << sumScore / m_particles.size());
}

inline void GridSlamProcessor::normalize()
{
  // normalize the log m_weights
  double gain = 1. / (m_obsSigmaGain * m_particles.size());
  double lmax = -std::numeric_limits<double>::max();

  for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
  {
    lmax = (*it)->weight > lmax ? (*it)->weight : lmax;
  }

  // cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;

  m_weights.clear();
  double wcum = 0;
  m_neff = 0;

  for (auto it = m_particles.begin(); it != m_particles.end(); it++)
  {
    m_weights.push_back(exp(gain * ((*it)->weight - lmax)));
    wcum += m_weights.back();

    // cout << "l=" << it->weight<< endl;
  }

  m_neff = 0;

  for (std::vector<double>::iterator it = m_weights.begin(); it != m_weights.end(); it++)
  {
    *it = *it / wcum;
    double w = *it;
    m_neff += w * w;
  }
  m_neff = 1. / m_neff;
}

inline bool GridSlamProcessor::resample(const double *plainReading, int adaptSize, const RangeReading *reading)
{
  bool hasResampled = false;

  if (m_neff < m_resampleThreshold * m_particles.size())
  {

    ROS_INFO_STREAM("*************RESAMPLE***************");

    uniform_resampler<double, double> resampler;

    m_indexes = resampler.resampleIndexes(m_weights, adaptSize);

    onResampleUpdate();

    // BEGIN: BUILDING TREE
    ParticleVector temp;
    unsigned int j = 0;
    std::vector<unsigned int> deletedParticles; // this is for deleteing
                                                // the
                                                // particles which have
                                                // been
                                                // resampled away.

    //		cerr << "Existing Nodes:" ;
    for (unsigned int i = 0; i < m_indexes.size(); i++)
    {
      //			cerr << " " << m_indexes[i];
      while (j < m_indexes[i])
      {
        deletedParticles.push_back(j);
        j++;
      }

      if (j == m_indexes[i])
        j++;
      temp.push_back(m_particles[m_indexes[i]]);

    }

    while (j < m_indexes.size())
    {
      deletedParticles.push_back(j);
      j++;
    }

    for (unsigned int i = 0; i < deletedParticles.size(); i++)
    {
      std::cerr << " " << deletedParticles[i];
//        delete m_particles[deletedParticles[i]].node;
//        m_particles[deletedParticles[i]].node = 0;
    }
    std::cerr << " Done" << std::endl;

    // END: BUILDING TREE
    std::cerr << "Deleting old particles...";
    m_particles.clear();
    std::cerr << "Done" << std::endl;
    std::cerr << "Copying Particles and  Registering  scans...";

    for (ParticleVector::iterator it = temp.begin(); it != temp.end(); it++)
    {
      m_particles.push_back(std::make_shared<Particle>(**it));
      m_particles.back()->setWeight(0);
      m_matcher.registerCells(m_particles.back(), plainReading);
    }
    std::cerr << " Done" << std::endl;
    hasResampled = true;
  }
  else
  {
    int index = 0;
    std::cerr << "Registering Scans:";
//      TNodeVector::iterator node_it = oldGeneration.begin();

    for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
    {
      m_matcher.registerCells(*it, plainReading);
    }
    std::cerr << "Done" << std::endl;
  }

  // END: BUILDING TREE

  return hasResampled;
}

inline void GridSlamProcessor::updateRefMap()
{
  for (int x = 0; x < m_refMap_ptr->getMapSizeX(); x++)
  {
    for (int y = 0; y < m_refMap_ptr->getMapSizeY(); y++)
    {
      m_refMap_ptr->cell(x, y).updateNew(no);
    }
  }
}

}
;

#endif /* ifndef GRIDSLAMPROCESSOR_H */
