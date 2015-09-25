#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "ros/console.h"
#include "smmap.h"
#include <gmapping/utils/macro_params.h>
#include <gmapping/utils/stat.h>
#include <iostream>
#include <gmapping/utils/gvalues.h>
#include <gmapping/particlefilter/Particle.h>

#define LASER_MAXBEAMS 2048

// GCC Bugfix
//#define isnan(x) (x == FP_NAN)

namespace GMapping
{
class ScanMatcher
{
public:

  typedef Covariance3 CovarianceMatrix;

  ScanMatcher();
  ~ScanMatcher();

//  double optimize(OrientedPoint & pnew, const ScanMatcherMap& map, const OrientedPoint & p, const double *readings,
//                  const SmUnorderedMap& active) const;

  double optimize(OrientedPoint & pnew, const ScanMatcherMap& map,
                  const OrientedPoint & p, const double *readings,
                  const SmPointerMap& p_map) const;

  void setLaserParameters(unsigned int beams, double *angles,
                          const OrientedPoint& lpose);
  void setMatchingParameters(std::shared_ptr<const ScanMatcherMap> map,
                             double urange, double range, double sigma,
                             int kernsize, double lopt, double aopt,
                             int iterations, double likelihoodSigma = 1,
                             unsigned int likelihoodSkip = 0);

  void registerCells(PointUnoSet & seenCells, SmUnorderedMap & activeCells,
                     const ScanMatcherMap& map, const OrientedPoint & p,
                     const double *readings);

  void registerCells(const std::shared_ptr<Particle> particle,
                     const double *readings);

//  inline double score(const ScanMatcherMap& map, const OrientedPoint & p, const double *readings,
//                      const SmUnorderedMap& active) const;

  inline double score(const ScanMatcherMap& map, const OrientedPoint & p,
                      const double *readings, const SmPointerMap& p_map) const;

  inline unsigned int likelihoodAndScore(double & s, double & l,
                                         const ScanMatcherMap& map,
                                         const OrientedPoint & p,
                                         const double *readings,
                                         const SmPointerMap& p_map) const;

//  inline unsigned int likelihoodAndScore(double & s, double & l, const ScanMatcherMap& map, const OrientedPoint & p,
//                                         const double *readings, const SmUnorderedMap& active) const;

  inline const double* laserAngles() const
  {
    return m_laserAngles;
  }

  inline unsigned int laserBeams() const
  {
    return m_laserBeams;
  }

  static const double nullLikelihood;

protected:

  // state of the matcher
  bool m_activeAreaComputed;

  /**laser parameters*/
  unsigned int m_laserBeams;
  double m_laserAngles[LASER_MAXBEAMS];

  // OrientedPoint m_laserPose;
PARAM_SET_GET(OrientedPoint, laserPose, protected, public, public)PARAM_SET_GET(double,
    laserMaxRange,
  protected,
  public,
  public)

  /**scan_matcher parameters*/
PARAM_SET_GET(double, usableRange, protected, public, public)PARAM_SET_GET(double,
    gaussianSigma,
  protected,
  public,
  public)PARAM_SET_GET(double,
    likelihoodSigma,
  protected,
  public,
  public)PARAM_SET_GET(int,
    kernelSize,
  protected,
  public,
  public)PARAM_SET_GET(double, optAngularDelta, protected, public, public)PARAM_SET_GET(double,
    optLinearDelta,
  protected,
  public,
  public)PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)PARAM_SET_GET(unsigned int,
    likelihoodSkip,
  protected,
  public,
  public)PARAM_SET_GET(double, llsamplerange, protected, public, public)PARAM_SET_GET(double,
    llsamplestep,
  protected,
  public,
  public)PARAM_SET_GET(double, lasamplerange, protected, public, public)PARAM_SET_GET(double,
    lasamplestep,
  protected,
  public,
  public)PARAM_SET_GET(double,
    enlargeStep,
  protected,
  public,
  public)PARAM_SET_GET(double, fullnessThreshold, protected, public, public)PARAM_SET_GET(double,
    angularOdometryReliability,
  protected,
  public,
  public)PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)PARAM_SET_GET(double,
    freeCellRatio,
  protected,
  public,
  public)PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)

  // allocate this large array only once
  IntPoint * m_linePoints;

private:
  std::shared_ptr<const ScanMatcherMap> map_;
};

//inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double *readings,
//                                 const SmUnorderedMap& active) const
//{
//
////  std::clock_t begin = std::clock();
//
//  double s = 0;
//  const double *angle = m_laserAngles + m_initialBeamsSkip;
//  OrientedPoint lp = p;
//
//  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
//  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
//  lp.theta += m_laserPose.theta;
//  unsigned int skip = 0;
//  double freeDelta = map.getDelta() * m_freeCellRatio;
//
//  for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
//  {
//    skip++;
//    skip = skip > m_likelihoodSkip ? 0 : skip;
//
//    if (skip || (*r > m_usableRange) || (*r == 0.0))
//      continue;
//    Point phit = lp;
//    phit.x += *r * cos(lp.theta + *angle);
//    phit.y += *r * sin(lp.theta + *angle);
//    IntPoint iphit = map.world2map(phit);
//    Point pfree(-freeDelta * cos(lp.theta + *angle), -freeDelta * sin(lp.theta + *angle));
//    IntPoint ipfree = map.world2map(phit + pfree) - iphit;
//    bool found = false;
//    Point bestMu(0., 0.);
//
//    for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
//      for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++)
//      {
//        IntPoint pr = iphit + IntPoint(xx, yy);
//        IntPoint pf = pr + ipfree;
//
//        // AccessibilityState s=map.storage().cellState(pr);
//        // if (s&Inside && s&Allocated){
//        // #TODO:0 reinitilizing is a poor solution...
////        const PointAccumulator& cell = map.cell(pr);
////        auto it = active.find(pr);
////        if (it != active.end())
////        {
////          &cell = *it->second;
////        }
//        auto it = active.find(pr);
//        const PointAccumulator& cell = it == active.end() ? map.cell(pr) : *it->second;
//
//        it = active.find(pf);
//        const PointAccumulator& fcell = it == active.end() ? map.cell(pf) : *it->second;
////        const PointAccumulator& fcell = map.cell(pf);
////        it = active.find(pf);
////        if (it != active.end())
////        {
////          &fcell = *it->second;
////        }
//
//        //std::cerr << "Hit " <<  (double)cell << " Miss: " << (double)fcell << std::endl;
//
//        if ((((double)cell) > m_fullnessThreshold) && (((double)fcell) < m_fullnessThreshold))
//        {
//          // std::cerr << "I have a hit! " <<  std::endl;
//          Point mu = phit - cell.mean();
//
//          if (!found)
//          {
//            bestMu = mu;
//            found = true;
//          }
//          else
//            bestMu = (mu * mu) < (bestMu * bestMu) ? mu : bestMu;
//        }
//
//        // }
//      }
//
//    if (found)
//      s += exp(-1. / m_gaussianSigma * bestMu * bestMu);
//  }
//
////  std::clock_t end = std::clock();
////  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
////  std::cerr << elapsed_secs << std::endl;
//
//  return s;
//}

inline double ScanMatcher::score(const ScanMatcherMap& map,
                                 const OrientedPoint& p, const double *readings,
                                 const SmPointerMap& p_map) const
{

//  std::clock_t begin = std::clock();

  double s = 0;
  const double *angle = m_laserAngles + m_initialBeamsSkip;
  OrientedPoint lp = p;

  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
  lp.theta += m_laserPose.theta;
  unsigned int skip = 0;
  double freeDelta = map.getDelta() * m_freeCellRatio;

  for (const double *r = readings + m_initialBeamsSkip;
      r < readings + m_laserBeams; r++, angle++)
  {
    skip++;
    skip = skip > m_likelihoodSkip ? 0 : skip;

    if (skip || (*r > m_usableRange) || (*r == 0.0))
      continue;
    Point phit = lp;
    phit.x += *r * cos(lp.theta + *angle);
    phit.y += *r * sin(lp.theta + *angle);
    IntPoint iphit = map.world2map(phit);
    Point pfree(-freeDelta * cos(lp.theta + *angle),
                -freeDelta * sin(lp.theta + *angle));
    IntPoint ipfree = map.world2map(phit + pfree) - iphit;
    bool found = false;
    Point bestMu(0., 0.);

    for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
      for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++)
      {
        IntPoint pr = iphit + IntPoint(xx, yy);
        IntPoint pf = pr + ipfree;

        // AccessibilityState s=map.storage().cellState(pr);
        // if (s&Inside && s&Allocated){
        // #TODO:0 reinitilizing is a poor solution...
//        const PointAccumulator& cell = map.cell(pr);
//        auto it = active.find(pr);
//        if (it != active.end())
//        {
//          &cell = *it->second;
//        }
        const PointAccumulator& cell = *p_map[pr.x][pr.y];
//
//        if (cell.Q[0] == 1)
//          ROS_INFO("HIT");

        const PointAccumulator& fcell = *p_map[pf.x][pf.y];

//        const PointAccumulator& fcell = map.cell(pf);
//        it = active.find(pf);
//        if (it != active.end())
//        {
//          &fcell = *it->second;
//        }

        //std::cerr << "Hit " <<  (double)cell << " Miss: " << (double)fcell << std::endl;

        if ((((double)cell) > m_fullnessThreshold)
            && (((double)fcell) < m_fullnessThreshold))
        {
          // std::cerr << "I have a hit! " <<  std::endl;
          Point mu = phit - cell.mean();

          if (!found)
          {
            bestMu = mu;
            found = true;
          }
          else
            bestMu = (mu * mu) < (bestMu * bestMu) ? mu : bestMu;
        }

        // }
      }

    if (found)
      s += exp(-1. / m_gaussianSigma * bestMu * bestMu);
  }

//  std::clock_t end = std::clock();
//  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//  std::cerr << elapsed_secs << std::endl;

  return s;
}

//inline unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map,
//                                                    const OrientedPoint& p, const double *readings,
//                                                    const SmUnorderedMap& active) const
//{
//  using namespace std;
//
//  l = 0;
//  s = 0;
//  const double *angle = m_laserAngles + m_initialBeamsSkip;
//  OrientedPoint lp = p;
//  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
//  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
//  lp.theta += m_laserPose.theta;
//  double noHit = nullLikelihood / (m_likelihoodSigma);
//  unsigned int skip = 0;
//  unsigned int c = 0;
//  double freeDelta = map.getDelta() * m_freeCellRatio;
//
//  for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
//  {
//    skip++;
//    skip = skip > m_likelihoodSkip ? 0 : skip;
//
//    if (*r > m_usableRange)
//      continue;
//
//    if (skip)
//      continue;
//    Point phit = lp;
//    phit.x += *r * cos(lp.theta + *angle);
//    phit.y += *r * sin(lp.theta + *angle);
//    IntPoint iphit = map.world2map(phit);
//    Point pfree(-freeDelta * cos(lp.theta + *angle), -freeDelta * sin(lp.theta + *angle));
//    IntPoint ipfree = map.world2map(phit + pfree) - iphit;
//    bool found = false;
//    Point bestMu(0., 0.);
//
//    for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
//      for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++)
//      {
//        IntPoint pr = iphit + IntPoint(xx, yy);
//        IntPoint pf = pr + ipfree;
//
//        // AccessibilityState s=map.storage().cellState(pr);
//        // if (s&Inside && s&Allocated){
//        auto it = active.find(pr);
//        const PointAccumulator& cell = it == active.end() ? map.cell(pr) : *it->second;
//
//        it = active.find(pf);
//        const PointAccumulator& fcell = it == active.end() ? map.cell(pf) : *it->second;
//
//        if ((((double)cell) > m_fullnessThreshold) && (((double)fcell) < m_fullnessThreshold))
//        {
//          Point mu = phit - cell.mean();
//
//          if (!found)
//          {
//            bestMu = mu;
//            found = true;
//          }
//          else
//            bestMu = (mu * mu) < (bestMu * bestMu) ? mu : bestMu;
//        }
//
//        // }
//      }
//
//    if (found)
//    {
//      s += exp(-1. / m_gaussianSigma * bestMu * bestMu);
//      c++;
//    }
//
//    if (!skip)
//    {
//      double f = (-1. / m_likelihoodSigma) * (bestMu * bestMu);
//      l += (found) ? f : noHit;
//    }
//  }
//  return c;
//}

inline unsigned int ScanMatcher::likelihoodAndScore(
    double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p,
    const double *readings, const SmPointerMap& p_map) const
{
  using namespace std;

  l = 0;
  s = 0;
  int n=0;
  int hit =0;
  const double *angle = m_laserAngles + m_initialBeamsSkip;
  OrientedPoint lp = p;
  lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
  lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
  lp.theta += m_laserPose.theta;
  double noHit = nullLikelihood / (m_likelihoodSigma);
  unsigned int skip = 0;
  unsigned int c = 0;
  double freeDelta = map.getDelta() * m_freeCellRatio;

  for (const double *r = readings + m_initialBeamsSkip;
      r < readings + m_laserBeams; r++, angle++)
  {
    n++;
    skip++;
    skip = skip > m_likelihoodSkip ? 0 : skip;

    if (*r > m_usableRange)
      continue;

    if (skip)
      continue;
    Point phit = lp;
    phit.x += *r * cos(lp.theta + *angle);
    phit.y += *r * sin(lp.theta + *angle);
    IntPoint iphit = map.world2map(phit);
    Point pfree(-freeDelta * cos(lp.theta + *angle),
                -freeDelta * sin(lp.theta + *angle));
    IntPoint ipfree = map.world2map(phit + pfree) - iphit;
    bool found = false;
    Point bestMu(0., 0.);

    for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
      for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++)
      {
        IntPoint pr = iphit + IntPoint(xx, yy);
        IntPoint pf = pr + ipfree;

        // AccessibilityState s=map.storage().cellState(pr);
        // if (s&Inside && s&Allocated){
        const PointAccumulator& cell = *p_map[pr.x][pr.y];

        const PointAccumulator& fcell = *p_map[pf.x][pf.y];

        if ((((double)cell) > m_fullnessThreshold)
            && (((double)fcell) < m_fullnessThreshold))
        {
          Point mu = phit - cell.mean();
          hit++;

          if (!found)
          {
            bestMu = mu;
            found = true;
          }
          else
            bestMu = (mu * mu) < (bestMu * bestMu) ? mu : bestMu;
        }

        // }
      }

    if (found)
    {
      s += exp(-1. / m_gaussianSigma * bestMu * bestMu);
      c++;
    }

    if (!skip)
    {
      double f = (-1. / m_likelihoodSigma) * (bestMu * bestMu);
      l += (found) ? f : noHit;
    }
  }
//  ROS_INFO_STREAM("n: " << n << " hit: " << hit << " weight: " << l << " score: " << s);
  return c;
}

}
;

#endif /* ifndef SCANMATCHER_H */
