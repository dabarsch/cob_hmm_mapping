#ifndef PARTICLE_H
#define PARTICLE_H

#include <gmapping/scanmatcher/smmap.h>

namespace GMapping
{
class Particle
{
public:
  Particle(const ScanMatcherMap& m) :
      map(m), pose(0, 0, 0), weight(0), weightSum(0), gweight(0), previousIndex(0),
      p_map_(boost::extents[m.getMapSizeX()][m.getMapSizeY()])
  {}

  /** @returns the weight of a particle */
  inline operator double() const
  {
    return weight;
  }

  /** @returns the pose of a particle */
  inline operator OrientedPoint() const
  {
    return pose;
  }

  /** sets the weight of a particle
   @param w the weight
   */
  inline void setWeight(double w)
  {
    weight = w;
  }

  /** The map */
  ScanMatcherMap map;

  /** The pose of the robot */
  OrientedPoint pose;

  /** The pose of the robot at the previous time frame (used for computing
   thr odometry displacements) */
  OrientedPoint previousPose;

  /** The weight of the particle */
  double weight;

  /** The cumulative weight of the particle */
  double weightSum;

  double gweight;

  /** The index of the previous particle in the trajectory tree */
  int previousIndex;

  /** Entry to the trajectory tree */
//      TNode *node;
  SmUnorderedMap activeCells;
  PointUnoSet seenCells;
  SmPointerMap p_map_;

  inline PointAccumulator& getCell(std::shared_ptr<ScanMatcherMap> refMap, IntPoint p) const
  {
    auto it = activeCells.find(p);
    if (it == activeCells.end())
    {
      return refMap->cell(p);
    }
    else
    {
      return *it->second;
    }
  }

  inline double getOcc(IntPoint p) const
  {
    return map.cell(p);
  }
private:
  Particle()
  {}
};
typedef std::vector<Particle> ParticleVector;
};



#endif /* ifndef PARTICLE_H */
