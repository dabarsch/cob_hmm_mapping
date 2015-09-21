#include <gmapping/scanmatcher/smmap.h>
#include <cassert>

namespace GMapping
{

const PointAccumulator& PointAccumulator::Unknown()
{
  if (!unknown_ptr)
    unknown_ptr = new PointAccumulator;
  return *unknown_ptr;
}

PointAccumulator* PointAccumulator::unknown_ptr = 0;

void PointAccumulator::updateGamma(int measure)
{
  double denom(0);
  for (int l = 0; l < N; l++)
  {
    for (int h = 0; h < N; h++)
    {
      Gamma[l][h] = A[l][h] * B[h][measure];
      denom += Gamma[l][h] * Q[l];
    }
  }
  for (int l = 0; l < N; l++)
  {
    for (int h = 0; h < N; h++)
    {
      Gamma[l][h] /= denom;
    }
  }
}

void PointAccumulator::updatePhi(int measure)
{
  auto OldPhi = Phi;
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      for (int k = 0; k < M; k++)
      {
        for (int h = 0; h < N; h++)
        {
          double sum = 0;
          for (int l = 0; l < N; l++)
          {
            double OldPhiL = OldPhi[i][j][k][l];
            if (measure == k && i == l && j == h)
            {
              sum += Gamma[l][h] * (OldPhiL + eta * (Q[l] - OldPhiL));
            }
            else
            {
              sum += Gamma[l][h] * OldPhiL * (1 - eta);
            }
          }
          Phi[i][j][k][h] = sum;
        }
      }
    }
  }
}

void PointAccumulator::updateQ()
{
  auto OldQ = Q;
  for (int l = 0; l < N; l++)
  {
    Q[l] = 0;
    for (int m = 0; m < N; m++)
    {
      Q[l] += Gamma[m][l] * OldQ[m];
    }
  }
}

void PointAccumulator::updateA()
{
  for (int i = 0; i < N; i++)
  {
    double denom = 0;
    std::array<double, 2> sum = {0, 0};
    assert(sum[0] == 0 && sum[1] == 0 && "sum not initialized with 0");
    for (int j = 0; j < N; j++)
    {
      for (int k = 0; k < M; k++)
      {
        for (int h = 0; h < N; h++)
        {
          sum[j] += Phi[i][j][k][h];
        }
      }
      denom += sum[j];
    }
    for (int j = 0; j < N; j++)
    {
      A[i][j] = sum[j] / denom;
    }
  }
}

bool PointAccumulator::updateNew(Obs obs, const Point& p)
{
  int measure;
  switch (obs)
  {
    case hit:
      updated = true;
      acc.x += static_cast<float>(p.x);
      acc.y += static_cast<float>(p.y);
      n++;
      visits += SIGHT_INC;
      measure = 0;
      updateGamma(measure);
      updatePhi(measure);
      updateQ();
      updateA();
      calcStatio();
      break;

    case miss:
      visits++;
      updated = true;
      measure = 1;
      updateGamma(measure);
      updatePhi(measure);
      updateQ();
      updateA();
      calcStatio();
      break;

    case no:
      measure = 2;
      updateGamma(measure);
      updatePhi(measure);
      updateQ();
      break;
  }

  return abs(A[0][1] - statio) <= 0.01;
}

void PointAccumulator::setfromMsg(const server_slam::PointAccumulator& msg)
{
  Q[0] = msg.occ;
  Q[1] = 1 - Q[0];

  acc.x = msg.acc[0];
  acc.y = msg.acc[1];
  A[0][1] = msg.a[0];
  A[0][0] = 1 - A[0][1];
  A[1][0] = msg.a[1];
  A[1][1] = 1 - A[1][0];
  statio = msg.statio;

  for (int i = 0; i != N; i++)
  {
    for (int j = 0; j != M; j++)
    {
      for (int k = 0; k != N; k++)
      {
        for (int l = 0; l != N; l++)
        {
          Phi[i][j][k][l] = msg.phi[l + k * 2 + j * 4 + i * 12];
        }
      }
    }
  }


  for (int i = 0; i != N; i++)
  {
    for (int j = 0; j != N; j++)
    {
      Gamma[i][j] = msg.gamma[j + 2 * i];
    }
  }

  n = msg.n;
  visits = msg.visits;
}

void PointAccumulator::setMsg(server_slam::PointAccumulator& msg)
{
  msg.occ = Q[0];

  msg.acc = std::vector<float> {acc.x, acc.y};

  msg.a = std::vector<double> {A[0][1], A[1][0]};

  msg.phi = std::vector<double> { Phi[0][0][0][0], Phi[0][0][0][1],
                                  Phi[0][0][1][0], Phi[0][0][1][1],
                                  Phi[0][1][0][0], Phi[0][1][0][1],
                                  Phi[0][1][1][0], Phi[0][1][1][1],
                                  Phi[0][2][0][0], Phi[0][2][0][1],
                                  Phi[0][2][1][0], Phi[0][2][1][1],
                                  Phi[1][0][0][0], Phi[1][0][0][1],
                                  Phi[1][0][1][0], Phi[1][0][1][1],
                                  Phi[1][1][0][0], Phi[1][1][0][1],
                                  Phi[1][1][1][0], Phi[1][1][1][1],
                                  Phi[1][2][0][0], Phi[1][2][0][1],
                                  Phi[1][2][1][0], Phi[1][2][1][1]  };


  msg.gamma = std::vector<double> { Gamma[0][0], Gamma[0][1],
                                    Gamma[1][0], Gamma[1][1]  };

  msg.n = n;

  msg.visits = visits;

}
}
;
