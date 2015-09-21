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

//  ROS_DEBUG_STREAM_NAMED("newParticle", "got the poses");

  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
  lp.theta += m_laserPose.theta;
//  ROS_DEBUG_STREAM_NAMED("newParticle", "before world to map");
  IntPoint p0 = map_->world2map(lp);
//  ROS_DEBUG_STREAM_NAMED("newParticle", "after world to map");

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
    Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
    IntPoint p1 = map_->world2map(phit);

    // IntPoint linePoints[20000] ;
    GridLineTraversalLine line;
    line.points = m_linePoints;
    GridLineTraversal::gridLine(p0, p1, &line);

//    ROS_DEBUG_STREAM_NAMED("newParticle", "Calculated the line");

    for (int i = 0; i < line.num_points - 1; i++)
    {
      particle->updateCell(line.points[i], miss);
    }

    if (d < m_usableRange)
    {
      particle->updateCell(p1, hit, phit);
    }
  }

}

//void ScanMatcher::registerCells(PointUnoSet & seenCells, SmUnorderedMap & activeCells, const ScanMatcherMap& map,
//                                const OrientedPoint & p, const double *readings)
//{
//  OrientedPoint lp = p;
//
//  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
//  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
//  lp.theta += m_laserPose.theta;
//  IntPoint p0 = map.world2map(lp);
//
//  const double *angle = m_laserAngles + m_initialBeamsSkip;
//  double esum = 0;
//
//  for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
//    if (m_generateMap)
//    {
//      double d = *r;
//
//      if ((d > m_laserMaxRange) || (d == 0.0) || std::isnan(d))
//        continue;
//
//      if (d > m_usableRange)
//        d = m_usableRange;
//      Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
//      IntPoint p1 = map.world2map(phit);
//
//      // IntPoint linePoints[20000] ;
//      GridLineTraversalLine line;
//      line.points = m_linePoints;
//      GridLineTraversal::gridLine(p0, p1, &line);
//
//      for (int i = 0; i < line.num_points - 1; i++)
//      {
//        // PointAccumulator& cell = map.cell(line.points[i]);
//        // double e = -cell.entropy();
//
//        // HMM Extensifdson
//
//        seenCells.insert(line.points[i]);
//
//        auto it = activeCells.find(line.points[i]);
//        if (it == activeCells.end())
//        {
//          std::shared_ptr<PointAccumulator> newCell = std::make_shared<PointAccumulator>(map.cell(line.points[i]));
//          newCell->updateNew(miss, Point(0, 0));
//          activeCells.insert( {line.points[i], newCell});
//        }
//        else
//        {
//          it->second->updateNew(miss, Point(0, 0));
//        }
//
//      }
//
//      if (d < m_usableRange)
//      {
//        // HMM Extensifdson
//
//        seenCells.insert(p1);
//
//        auto it = activeCells.find(p1);
//        if (it == activeCells.end())
//        {
//          std::shared_ptr<PointAccumulator> newCell = std::make_shared<PointAccumulator>(map.cell(p1));
//          newCell->updateNew(hit, phit);
//          activeCells.insert( {p1, newCell});
//        }
//        else
//        {
//          it->second->updateNew(hit, phit);
//        }
//
//      }
//    }
//    else
//    {
//      if ((*r > m_laserMaxRange) || (*r > m_usableRange) || (*r == 0.0) || std::isnan(*r))
//        continue;
//      Point phit = lp;
//      phit.x += *r * cos(lp.theta + *angle);
//      phit.y += *r * sin(lp.theta + *angle);
//      IntPoint p1 = map.world2map(phit);
//      assert(p1.x >= 0 && p1.y >= 0);
//
//      // HMM Extensison
//
//      seenCells.insert(p1);
//
//      auto it = activeCells.find(p1);
//      if (it == activeCells.end())
//      {
//        std::shared_ptr<PointAccumulator> newCell = std::make_shared<PointAccumulator>(map.cell(p1));
//        newCell->updateNew(hit, phit);
//        activeCells.insert( {p1, newCell});
//      }
//      else
//      {
//        it->second->updateNew(hit, phit);
//      }
//
////      if (activeCells.count(p1) == 0)
////      {
////        std::shared_ptr<PointAccumulator> newCell = std::make_shared<PointAccumulator>(map.cell(p1));
////        newCell->updateNew(hit, phit);
////        activeCells.insert( {p1, newCell});
////      }
////      else
////      {
////        assert(activeCells.count(p1) == 1);
////        activeCells.at(p1)->updateNew(hit, phit);
////      }
//    }
//
//  for (auto it = activeCells.begin(); it != activeCells.end(); it++)
//  {
//    if (it->second->updated)
//    {
//      it->second->updated = false;
//    }
//    else
//    {
//      it->second->updateNew(no);
//    }
//  }
//
//  // while(it!= activeCells.end())
//  // {
//  //   if (it->second->toDelete())
//  //   {
//  //     it = activeCells.erase(it);
//  //   }
//  //   else
//  //   {
//  //     it++;
//  //   }
//  // }
//}

double ScanMatcher::optimize(OrientedPoint & pnew, const ScanMatcherMap& map, const OrientedPoint & init,
                             const double *readings, const SmUnorderedMap& active) const
{
  double bestScore = -1;
  OrientedPoint currentPose = init;
  double currentScore = score(map, currentPose, readings, active);
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
      double localScore = odo_gain * score(map, localPose, readings, active);

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
