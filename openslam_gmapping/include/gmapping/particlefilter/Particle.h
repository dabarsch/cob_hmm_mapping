#ifndef PARTICLE_H
#define PARTICLE_H

#include <gmapping/scanmatcher/smmap.h>

namespace GMapping
{
class Particle
{
public:

  //Functions
  Particle(const ScanMatcherMap& m);
  inline operator double() const;
  inline operator OrientedPoint() const;
  inline void setWeight(double w);
  inline PointAccumulator& getCell(std::shared_ptr<ScanMatcherMap> refMap, IntPoint p) const;

  //Member
  OrientedPoint pose;
  double weight;
  double weightSum;
  double gweight;
  SmUnorderedMap activeCells;
  PointUnoSet seenCells;
  SmPointerMap p_map_;
};

typedef std::vector<Particle> ParticleVector;

//Definitions
inline Particle::Particle(const ScanMatcherMap& m) :
     pose(0, 0, 0), weight(0), weightSum(0), gweight(0),
     p_map_(boost::extents[m.getMapSizeX()][m.getMapSizeY()])
{}

inline Particle::operator double() const
{
  return weight;
}

inline Particle::operator OrientedPoint() const
{
  return pose;
}

inline void Particle::setWeight(double w)
{
  weight = w;
}

inline PointAccumulator& Particle::getCell(std::shared_ptr<ScanMatcherMap> refMap, IntPoint p) const
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

};

#endif /* ifndef PARTICLE_H */
