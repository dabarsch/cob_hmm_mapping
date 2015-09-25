#ifndef PARTICLE_H
#define PARTICLE_H

//Ros includes
#include <ros/assert.h>

//STL includes
#include <memory>

//Project includes
#include <gmapping/scanmatcher/smmap.h>

namespace GMapping
{

class Particle
{

public:
  // public functions
  Particle(const ScanMatcherMap& m);
  inline operator double() const;
  inline operator OrientedPoint() const;
  inline void setWeight(double w);
  inline PointAccumulator& getCell(std::shared_ptr<ScanMatcherMap> refMap, IntPoint p) const;
  inline void updateCell(const IntPoint& p, Obs obs, const Point& pHit = {0, 0});
  inline OrientedPoint getPose();
  inline void updateRemaining();
  inline void removeActive(const IntPoint& p);
  inline void initPtrMap();

  // public member
  OrientedPoint pose;
  double weight;
  double weightSum;
  double gweight;
  SmUnorderedMap activeCells;
  PointUnoSet seenCells;
  SmPointerMap p_map_;
  std::shared_ptr<const ScanMatcherMap> ref_map_ptr_;
  const ScanMatcherMap& map_;

private:
  // private functions
  inline std::shared_ptr<PointAccumulator> insertActive(const IntPoint& p);

  // private member
};

typedef std::vector<std::shared_ptr<Particle>> ParticleVector;

inline OrientedPoint Particle::getPose()
{
  return pose;
}

inline Particle::Particle(const ScanMatcherMap& m) :
    map_(m), pose(0, 0, 0), weight(0), weightSum(0), gweight(0), p_map_(boost::extents[m.getMapSizeX()][m.getMapSizeY()])
{
  initPtrMap();
}

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

inline void Particle::updateCell(const IntPoint& p, Obs obs, const Point& pHit)
{
  // no-observation must not call this function!
  ROS_ASSERT(obs == hit || obs == miss);

  // add to seen cells
  seenCells.insert(p);

  // get correct poiter to cell
  auto pa_ptr = insertActive(p);

  // update cell
  pa_ptr->updateNew(obs, pHit);
}

inline void Particle::removeActive(const IntPoint& p)
{
  // check if element existed before
  if (static_cast<bool>(activeCells.erase(p)))
  {
    p_map_[p.x][p.y] = &map_.cell(p);
  }
}

inline std::shared_ptr<PointAccumulator> Particle::insertActive(const IntPoint& p)
{
  // check if PA is already stored in map
  auto it = activeCells.find(p);

  if (it == activeCells.end())
  {
    // element has to be added

    // add element
    auto res = activeCells.insert( {p, std::make_shared<PointAccumulator>(map_.cell(p))});

    // check that element was really added
    ROS_ASSERT(res.second);

    // add to ptr map
    p_map_[p.x][p.y] = res.first->second.get();

    // return shared ptr
    return res.first->second;
  }
  else
  {
    // element is already in map

    //return shared ptr
    return it->second;
  }
}

inline void Particle::updateRemaining()
{
  for (auto it = activeCells.begin(); it != activeCells.end(); it++)
  {
    it->second->updateIfNotSeen();
  }
}

inline void Particle::initPtrMap()
{
  for (auto x = 0; x < map_.getMapSizeX(); x++)
  {
    for (auto y = 0; y < map_.getMapSizeY(); y++)
    {
      p_map_[x][y] = &map_.cell(x,y);
    }
  }
}

}
;

#endif /* ifndef PARTICLE_H */
