#ifndef SMMAP_H
#define SMMAP_H
#include <array>
#include <unordered_map>
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#include <server_slam/PointAccumulator.h>
#include <unordered_set>
#include <math.h>

#include <memory>

#define SIGHT_INC 1

namespace GMapping
{
enum Obs
{
  hit, miss, no
};

struct PointAccumulator
{
  static int const N = 2;
  static int const M = 3;
  //static double const e = 0.01;
  typedef point<float> FloatPoint;

  /* before
   PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
   */

  /*after begin*/
  PointAccumulator() :
      acc(0, 0), n(0), visits(0), switches(0), lastObs(no), A { { {0.8, 0.2}, {0.1, 0.9}}}, B {
          { {0.7, 0.2, 0.1}, {0.2, 0.7, 0.1}}},
          Phi { { { { { { {1e-10, 1e-10},
                          {1e-10, 1e-10},
                          {1e-10, 1e-10}}},
                      { { {1e-10, 1e-10},
                          {1e-10, 1e-10},
                          {1e-10, 1e-10}}}}},
                  { { { { {1e-10, 1e-10},
                          {1e-10, 1e-10},
                          {1e-10, 1e-10}}},
                      { { {1e-10, 1e-10},
                          {1e-10, 1e-10},
                          {1e-10, 1e-10}}}}}}},
                          Q { {
          0.5, 0.5}}, eta(0.001), Gamma { { {0, 0}, {0, 0}}}, updated(false), unseenCount(0), unseenLimit(3)
  {
  }

  PointAccumulator(int i) :
      acc(0, 0), n(0), visits(0), switches(0), lastObs(no), A { { {0.8, 0.2}, {0.1, 0.9}}}, B {
          { {0.7, 0.2, 0.1}, {0.2, 0.7, 0.1}}}, Phi { { { { { { {1e-10, 1e-10}, {1e-10, 1e-10}, {1e-10, 1e-10}}}, { {
          {1e-10, 1e-10}, {1e-10, 1e-10}, {1e-10, 1e-10}}}}},
                                                       { { { { {1e-10, 1e-10}, {1e-10, 1e-10}, {1e-10, 1e-10}}}, { {
                                                           {1e-10, 1e-10}, {1e-10, 1e-10}, {1e-10, 1e-10}}}}}}}, Q { {
          0.5, 0.5}}, eta(0.001), updated(false), unseenCount(0), unseenLimit(3)
  {//void PointAccumulator::calcSteps()
    assert(i == -1);
  }

  /*after end*/

  inline Point mean() const
  {
    if (n == 0)
      return Point(acc.x, acc.y);
    else
      return 1. / n * Point(acc.x, acc.y);
  }

  inline operator double() const
  {
    return visits ? Q[0] : -1;
  }

  inline void add(const PointAccumulator& p)
  {
    std::cout << "Ohhh shit a cell was added" << std::endl;

    exit(EXIT_FAILURE);
    acc = acc + p.acc;
    n += p.n;
    visits += p.visits;
  }

  inline bool toDelete()
  {
    if (updated)
    {
      updated = false;
      unseenCount = 0;
      return false;
    }
    else
    {
      return unseenLimit <= ++unseenCount;
    }
  }

  inline void update(bool value, const Point& p = Point(0, 0));
  bool updateNew(Obs obs, const Point& p = Point(0, 0));

  inline int getPof();
  inline int getPfo();
  void updateGamma(int measure);
  void updatePhi(int measure);
  void updateQ();
  void updateA();
  inline void calcStatio();
  inline void calcSteps();
  void setfromMsg(const server_slam::PointAccumulator& msg);
  void setMsg(server_slam::PointAccumulator& msg);

  static const PointAccumulator& Unknown();
  static PointAccumulator *unknown_ptr;
  FloatPoint acc;
  int n, visits, switches;
  inline double entropy() const;
  bool updated;
  std::array<double, N> Q;

private:

  std::array<std::array<double, N>, N> A;
  std::array<std::array<double, M>, N> B;
  std::array<std::array<std::array<std::array<double, N>, M>, N>, N> Phi;
  std::array<std::array<double, N>, N> Gamma;
  double statio;
  int steps;
  double eta;
  int unseenCount;
  int unseenLimit;
  Obs lastObs;
};

void PointAccumulator::calcStatio()
{
  statio = A[0][1] / (A[0][1] + A[1][0]);
}

//void PointAccumulator::calcSteps()
//{
//  steps = ceil(log(PointAccumulator::e / abs(A[0][1] - statio)) / log(abs(1 - A[0][1] - A[1][0]))) - 1;
//}

void PointAccumulator::update(bool value, const Point& p)
{
  updated = true;

  if (value)
  {
    acc.x += static_cast<float>(p.x);
    acc.y += static_cast<float>(p.y);
    n++;
    visits += SIGHT_INC;
  }
  else
    visits++;
  int measure = static_cast<int>(!value);
  updateGamma(measure);
  updatePhi(measure);
  updateQ();
  updateA();
}

double PointAccumulator::entropy() const
{
  if (!visits)
    return -log(.5);

  if ((n == visits) || (n == 0))
    return 0;

  double x = (double)n * SIGHT_INC / (double)visits;
  return -(x * log(x) + (1 - x) * log(1 - x));
}

int PointAccumulator::getPof()
{
  return visits ? (int)(A[1][0] * 100 + 0.5) : -1;
}

int PointAccumulator::getPfo()
{
  return visits ? (int)(A[0][1] * 100 + 0.5) : -1;
}

typedef Map<PointAccumulator, HierarchicalArray2D<PointAccumulator>> ScanMatcherMap;
typedef std::unordered_map<IntPoint, std::shared_ptr<PointAccumulator>> SmUnorderedMap;
typedef std::unordered_set<IntPoint> PointUnoSet;
}
;

#endif /* ifndef SMMAP_H */
