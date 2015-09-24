#include <cstring>
#include <limits>
#include <list>
#include <iostream>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include "gridlinetraversal.h"

// #define GENERATE_MAPS

namespace GMapping
{

const double ScanMatcher::nullLikelihood = -.5;

ScanMatcher::ScanMatcher() :
    m_laserPose(0, 0, 0)
{
  // m_laserAngles=0;
  m_laserBeams = 0;
  m_optRecursiveIterations = 3;
  m_activeAreaComputed = false;

  // This  are the dafault settings for a grid map of 5 cm
  m_llsamplerange = 0.01;
  m_llsamplestep = 0.01;
  m_lasamplerange = 0.005;
  m_lasamplestep = 0.005;
  m_enlargeStep = 10.;
  m_fullnessThreshold = 0.5;
  m_angularOdometryReliability = 0.;
  m_linearOdometryReliability = 0.;
  m_freeCellRatio = sqrt(2.);
  m_initialBeamsSkip = 0;

  /*
   // This  are the dafault settings for a grid map of 10 cm
   m_llsamplerange=0.1;
   m_llsamplestep=0.1;
   m_lasamplerange=0.02;
   m_lasamplestep=0.01;
   */

  // This  are the dafault settings for a grid map of 20/25 cm
  /*
   m_llsamplerange=0.2;
   m_llsamplestep=0.1;
   m_lasamplerange=0.02;
   m_lasamplestep=0.01;
   m_generateMap=false;
   */

  m_linePoints = new IntPoint[20000];
}

ScanMatcher::~ScanMatcher()
{
  delete[] m_linePoints;
}

void ScanMatcher::registerCells(const std::shared_ptr<Particle> particle, const double *readings)
{
  OrientedPoint lp = particle->getPose();
  OrientedPoint p = lp;


//  ROS_INFO_STREAM_NAMED("newParticle", "got the poses");

  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
  lp.theta += m_laserPose.theta;
//  ROS_INFO_STREAM_NAMED("newParticle", "before world to map");
//  ROS_INFO_STREAM_NAMED("newParticle", "lp x:" << lp.x <<  "lp y:" << lp.y);
  IntPoint p0 = map_->world2map(lp);
//  ROS_INFO_STREAM_NAMED("newParticle", "lp x:" << p0.x <<  "lp y:" << p0.y);
//  ROS_INFO_STREAM_NAMED("newParticle", "after world to map");

  const double *angle = m_laserAngles + m_initialBeamsSkip;
  double esum = 0;

//  ROS_DEBUG_STREAM_NAMED("newParticle", "before the loop");
  for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
  {
    double d = *r;

//    ROS_DEBUG_STREAM_NAMED("newParticle", "after the d");

    if ((d > m_laserMaxRange) || (d == 0.0) || std::isnan(d))
      continue;

    if (d > m_usableRange)
      d = m_usableRange;
//    ROS_INFO_STREAM_NAMED("newParticle", "before phit");
    Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
    IntPoint p1 = map_->world2map(phit);

//    ROS_INFO_STREAM_NAMED("newParticle", "after phit");

    // IntPoint linePoints[20000] ;
    GridLineTraversalLine line;
    line.points = m_linePoints;
    GridLineTraversal::gridLine(p0, p1, &line);

//    ROS_INFO_STREAM_NAMED("newParticle", "after line " << line.num_points);

//    ROS_DEBUG_STREAM_NAMED("newParticle", "Calculated the line");


    for (int i = 0; i < line.num_points - 1; i++)
    {
//      ROS_INFO_STREAM_NAMED("newParticle", "I will update a cell");
      particle->updateCell(line.points[i], miss);
    }

    if (d < m_usableRange)
    {
      particle->updateCell(p1, hit, phit);
    }
  }

}

double ScanMatcher::optimize(OrientedPoint & pnew, const ScanMatcherMap& map, const OrientedPoint & init,
                             const double *readings, const SmPointerMap& p_map) const
{
  double bestScore = -1;
  OrientedPoint currentPose = init;
  double currentScore = score(map, currentPose, readings, p_map);
  //std::cerr << "Startscore: " << currentScore << std::endl;
  double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
  unsigned int refinement = 0;
  enum Move
  {
    Front, Back, Left, Right, TurnLeft, TurnRight, Done
  };

  /*    cout << __PRETTY_FUNCTION__<<  " readings: ";
   for (int i=0; i<m_laserBeams; i++){
   cout << readings[i] << " ";
   }
   cout << endl;
   */
  int c_iterations = 0;

  do
  {
    if (bestScore >= currentScore)
    {
      refinement++;
      adelta *= .5;
      ldelta *= .5;
    }
    bestScore = currentScore;

    //          cout <<"score="<< currentScore << " refinement=" << refinement;
    //          cout <<  "pose=" << currentPose.x  << " " << currentPose.y << "
    // " << currentPose.theta << endl;
    OrientedPoint bestLocalPose = currentPose;
    OrientedPoint localPose = currentPose;

    Move move = Front;

    do
    {
      localPose = currentPose;

      switch (move)
      {
        case Front:
          localPose.x += ldelta;
          move = Back;
          break;

        case Back:
          localPose.x -= ldelta;
          move = Left;
          break;

        case Left:
          localPose.y -= ldelta;
          move = Right;
          break;

        case Right:
          localPose.y += ldelta;
          move = TurnLeft;
          break;

        case TurnLeft:
          localPose.theta += adelta;
          move = TurnRight;
          break;

        case TurnRight:
          localPose.theta -= adelta;
          move = Done;
          break;

        default:
          ;
      }

      double odo_gain = 1;

      if (m_angularOdometryReliability > 0.)
      {
        double dth = init.theta - localPose.theta;
        dth = atan2(sin(dth), cos(dth));
        dth *= dth;
        odo_gain *= exp(-m_angularOdometryReliability * dth);
      }

      if (m_linearOdometryReliability > 0.)
      {
        double dx = init.x - localPose.x;
        double dy = init.y - localPose.y;
        double drho = dx * dx + dy * dy;
        odo_gain *= exp(-m_linearOdometryReliability * drho);
      }
      double localScore = odo_gain * score(map, localPose, readings, p_map);

      if (localScore > currentScore)
      {
        currentScore = localScore;
        bestLocalPose = localPose;
      }
      c_iterations++;
    } while (move != Done);
    currentPose = bestLocalPose;

    //          cout << "currentScore=" << currentScore<< endl;
    // here we look for the best move;
  } while (currentScore > bestScore || refinement < m_optRecursiveIterations);
//  std::cerr << m_optRecursiveIterations << std::endl;

  // cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
  // cout << __PRETTY_FUNCTION__ << "iterations=" << c_iterations<< endl;
  pnew = currentPose;
  return bestScore;
}


//double ScanMatcher::optimize(OrientedPoint & pnew, const ScanMatcherMap& map, const OrientedPoint & init,
//                             const double *readings, const SmUnorderedMap& active) const
//{
//  double bestScore = -1;
//  OrientedPoint currentPose = init;
//  double currentScore = score(map, currentPose, readings, active);
//  //std::cerr << "Startscore: " << currentScore << std::endl;
//  double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
//  unsigned int refinement = 0;
//  enum Move
//  {
//    Front, Back, Left, Right, TurnLeft, TurnRight, Done
//  };
//
//  /*    cout << __PRETTY_FUNCTION__<<  " readings: ";
//   for (int i=0; i<m_laserBeams; i++){
//   cout << readings[i] << " ";
//   }
//   cout << endl;
//   */
//  int c_iterations = 0;
//
//  do
//  {
//    if (bestScore >= currentScore)
//    {
//      refinement++;
//      adelta *= .5;
//      ldelta *= .5;
//    }
//    bestScore = currentScore;
//
//    //          cout <<"score="<< currentScore << " refinement=" << refinement;
//    //          cout <<  "pose=" << currentPose.x  << " " << currentPose.y << "
//    // " << currentPose.theta << endl;
//    OrientedPoint bestLocalPose = currentPose;
//    OrientedPoint localPose = currentPose;
//
//    Move move = Front;
//
//    do
//    {
//      localPose = currentPose;
//
//      switch (move)
//      {
//        case Front:
//          localPose.x += ldelta;
//          move = Back;
//          break;
//
//        case Back:
//          localPose.x -= ldelta;
//          move = Left;
//          break;
//
//        case Left:
//          localPose.y -= ldelta;
//          move = Right;
//          break;
//
//        case Right:
//          localPose.y += ldelta;
//          move = TurnLeft;
//          break;
//
//        case TurnLeft:
//          localPose.theta += adelta;
//          move = TurnRight;
//          break;
//
//        case TurnRight:
//          localPose.theta -= adelta;
//          move = Done;
//          break;
//
//        default:
//          ;
//      }
//
//      double odo_gain = 1;
//
//      if (m_angularOdometryReliability > 0.)
//      {
//        double dth = init.theta - localPose.theta;
//        dth = atan2(sin(dth), cos(dth));
//        dth *= dth;
//        odo_gain *= exp(-m_angularOdometryReliability * dth);
//      }
//
//      if (m_linearOdometryReliability > 0.)
//      {
//        double dx = init.x - localPose.x;
//        double dy = init.y - localPose.y;
//        double drho = dx * dx + dy * dy;
//        odo_gain *= exp(-m_linearOdometryReliability * drho);
//      }
//      double localScore = odo_gain * score(map, localPose, readings, active);
//
//      if (localScore > currentScore)
//      {
//        currentScore = localScore;
//        bestLocalPose = localPose;
//      }
//      c_iterations++;
//    } while (move != Done);
//    currentPose = bestLocalPose;
//
//    //          cout << "currentScore=" << currentScore<< endl;
//    // here we look for the best move;
//  } while (currentScore > bestScore || refinement < m_optRecursiveIterations);
////  std::cerr << m_optRecursiveIterations << std::endl;
//
//  // cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
//  // cout << __PRETTY_FUNCTION__ << "iterations=" << c_iterations<< endl;
//  pnew = currentPose;
//  return bestScore;
//}

struct ScoredMove
{
  OrientedPoint pose;
  double score;
  double likelihood;
};

typedef std::list<ScoredMove> ScoredMoveList;

void ScanMatcher::setLaserParameters(unsigned int beams, double *angles, const OrientedPoint& lpose)
{
  /*if (m_laserAngles)
   delete [] m_laserAngles;
   */
  assert(beams < LASER_MAXBEAMS);
  m_laserPose = lpose;
  m_laserBeams = beams;

  // m_laserAngles=new double[beams];
  memcpy(m_laserAngles, angles, sizeof(double) * m_laserBeams);
}

void ScanMatcher::setMatchingParameters(std::shared_ptr<const ScanMatcherMap> map, double urange, double range, double sigma, int kernsize, double lopt,
                                        double aopt, int iterations, double likelihoodSigma,
                                        unsigned int likelihoodSkip)
{
  map_ = map;
  m_usableRange = urange;
  m_laserMaxRange = range;
  m_kernelSize = kernsize;
  m_optLinearDelta = lopt;
  m_optAngularDelta = aopt;
  m_optRecursiveIterations = iterations;
  m_gaussianSigma = sigma;
  m_likelihoodSigma = likelihoodSigma;
  m_likelihoodSkip = likelihoodSkip;
}
}
