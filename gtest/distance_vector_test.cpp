/**
 * Author: R-CO
 * E-Mail: daniel1820kobe@gmail.com
 **/
#include "distance_vector.hpp"
using namespace rco::dv;

#include <gtest/gtest.h>

TEST(Distance, int_type_test_less_than) {
  {
    constexpr int INF = -1;
    Distance<int, INF> d_1{1};
    Distance<int, INF> d_2{2};

    EXPECT_LT(d_1, d_2);
  }

  {
    constexpr int INF = -1;
    Distance<int, INF> d_1{1};
    Distance<int, INF> d_2{INF};

    EXPECT_LT(d_1, d_2);
  }
}

TEST(DistanceVector, test1) {
  constexpr int kNodeCount = 3;
  constexpr int kIFDis = -1;
  std::array<Node<int, kNodeCount, kIFDis>, kNodeCount> nodes;
  nodes[0].setDirectDv({0, 2, 3});  // A
  nodes[1].setDirectDv({2, 0, 4});  // B
  nodes[2].setDirectDv({3, 4, 0});  // C

  DistanceVectorSystem<int, kNodeCount, kIFDis> dv_sys(nodes);

  auto iteration_count = dv_sys.run();

  EXPECT_GT(iteration_count, 0);
}

TEST(DistanceVector, test2) {
  constexpr int kNodeCount = 4;
  constexpr int kIFDis = -1;
  std::array<Node<int, kNodeCount, kIFDis>, kNodeCount> nodes;
  nodes[0].setDirectDv({0, 2, kIFDis, 5});  // A
  nodes[1].setDirectDv({2, 0, 3, kIFDis});  // B
  nodes[2].setDirectDv({kIFDis, 3, 0, 4});  // C
  nodes[3].setDirectDv({5, kIFDis, 4, 0});  // D

  DistanceVectorSystem<int, kNodeCount, kIFDis> dv_sys(nodes);

  auto iteration_count = dv_sys.run();

  EXPECT_GT(iteration_count, 0);
}

TEST(DistanceVector, test3) {
  constexpr int kNodeCount = 5;
  constexpr int kIFDis = -1;
  std::array<Node<int, kNodeCount, kIFDis>, kNodeCount> nodes;
  nodes[0].setDirectDv({0, 7, kIFDis, kIFDis, 1});  // A
  nodes[1].setDirectDv({7, 0, 1, kIFDis, 8});       // B
  nodes[2].setDirectDv({kIFDis, 1, 0, 2, kIFDis});  // C
  nodes[3].setDirectDv({kIFDis, kIFDis, 2, 0, 2});  // D
  nodes[4].setDirectDv({1, 8, kIFDis, 2, 0});       // E
  // {0, 7, kIFDis, kIFDis, 1},
  // {7, 0, 1, kIFDis, 8},
  // {kIFDis, 1, 0, 2, kIFDis},
  // {kIFDis, kIFDis, 2, 0, 2},
  // {1, 8, kIFDis, 2, 0}}

  DistanceVectorSystem<int, kNodeCount, kIFDis> dv_sys(nodes);

  auto iteration_count = dv_sys.run();

  EXPECT_GT(iteration_count, 0);
}