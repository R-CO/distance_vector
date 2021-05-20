/**
 * Author: R-CO
 * E-Mail: daniel1820kobe@gmail.com
 **/
#include "distance_vector.hpp"
using namespace rco::dv;

#include <gtest/gtest.h>

TEST(DistanceVector, test) {
  constexpr int kNodeCount = 5;
  constexpr int kIFDis = -1;
  std::array<Node<kNodeCount, kIFDis>, kNodeCount> nodes;
  nodes[0].direct_distance_.setDistanceVector({0, 7, kIFDis, kIFDis, 1});
  nodes[1].direct_distance_.setDistanceVector({7, 0, 1, kIFDis, 8});
  nodes[2].direct_distance_.setDistanceVector({0, 7, kIFDis, kIFDis, 1});
  nodes[3].direct_distance_.setDistanceVector({0, 7, kIFDis, kIFDis, 1});
  nodes[4].direct_distance_.setDistanceVector({0, 7, kIFDis, kIFDis, 1});
  // {0, 7, kIFDis, kIFDis, 1},
  // {7, 0, 1, kIFDis, 8},
  // {kIFDis, 1, 0, 2, kIFDis},
  // {kIFDis, kIFDis, 2, 0, 2},
  // {1, 8, kIFDis, 2, 0}}

  DistanceVectorSystem<kNodeCount, kIFDis> dv_sys(nodes);

	auto iteration_count = dv_sys.run();

	EXPECT_GT(iteration_count, 0);
}
