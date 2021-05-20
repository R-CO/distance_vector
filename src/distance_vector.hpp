/**
 * Author: R-CO
 * E-Mail: daniel1820kobe@gmail.com
 **/
#ifndef DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP
#define DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP

#include <algorithm>
#include <array>
#include <exception>
#include <vector>
namespace rco {

namespace dv {

template <size_t NodeCount, int InfiniteDistance>
class DistanceVector {
 public:
  DistanceVector() { distance_.fill(InfiniteDistance); }

  DistanceVector(std::array<int, NodeCount> &distance) : distance_(distance) {}

  void setDistanceVector(const std::vector<int> &vec) {
    if (vec.size() < NodeCount) {
      throw std::runtime_error("size of vec is less than NodeCount\n");
    }

    for (auto i = 0; i < NodeCount; ++i) {
      distance_[i] = vec[i];
    }
  }

  void fillWithInfiniteDistance() { distance_.fill(InfiniteDistance); }

  auto begin() { return distance_.begin(); }
  auto begin() const { return distance_.begin(); }

  auto end() { return distance_.end(); }
  auto end() const { return distance_.end(); }

  std::array<int, NodeCount> distance_;
};

template <size_t NodeCount, int InfiniteDistance>
class Node {
 public:
  Node() {}

  Node(DistanceVector<NodeCount, InfiniteDistance> &direct_distance)
      : direct_distance_(direct_distance) {}

  void setDirectDv(const std::vector<int> &vec) {
    direct_distance_.setDistanceVector(vec);
  }

  void initDvBasedOnDirectDv() {
    std::for_each(
        dv_of_neighbors_.begin(), dv_of_neighbors_.end(),
        [](DistanceVector<NodeCount, InfiniteDistance> &distance_vector) {
          distance_vector.fillWithInfiniteDistance();
        });

    std::copy(direct_distance_.begin(), direct_distance_.end(),
              dv_of_neighbors_[id_].begin());
  }

  void sendDvToNeighbors(
      std::array<Node<NodeCount, InfiniteDistance>, NodeCount> &neighbors,
      bool &has_value_change) {
    for (auto &it : neighbors) {
      it.updateDvOfNodeX(this->id_, this->dv_of_neighbors_[id_],
                         has_value_change);
    }
  }

  void updateDvOfNodeX(
      const size_t id_v,
      const DistanceVector<NodeCount, InfiniteDistance> &dv_of_v,
      bool &has_value_changed) {
    // for (size_t y = 0; y < NodeCount; ++y) {
    //   auto &dx_y = dv_of_neighbors_[id_].distance_[y];
    //   auto dv_y = dv_of_v.distance_[y];
    //   auto cx_v = direct_distance_.distance_[id_v];
    //   if (dx_y == InfiniteDistance &&
    //       (cx_v == InfiniteDistance || dv_y == InfiniteDistance)) {
    //     continue;
    //   } else if (dx_y == InfiniteDistance) {
    //     dx_y = cx_v + dv_y;
    //     has_value_changed = true;
    //   } else if (dx_y != InfiniteDistance && cx_v != InfiniteDistance &&
    //              dv_y != InfiniteDistance) {
    //     dx_y = std::min(cx_v + dv_y, dx_y);
    //     has_value_changed = true;
    //   }
    // }
  }

  DistanceVector<NodeCount, InfiniteDistance> direct_distance_;
  std::array<DistanceVector<NodeCount, InfiniteDistance>, NodeCount>
      dv_of_neighbors_;

  // std::array<Node *, NodeCount> neighbors_;
  size_t id_;
};

template <size_t NodeCount, int InfiniteDistance>
class DistanceVectorSystem {
 public:
  // DistanceVectorSystem() { initNodes(); }

  DistanceVectorSystem(
      std::array<Node<NodeCount, InfiniteDistance>, NodeCount> &nodes)
      : nodes_(nodes) {
    initNodes();
  }

  size_t run() {
    bool has_value_changed = false;
    size_t iteration_count = 0;
    do {
      has_value_changed = false;
      ++iteration_count;
      for (size_t id = 0; id < NodeCount; ++id) {
        nodes_[id].sendDvToNeighbors(nodes_, has_value_changed);
      }
    } while (has_value_changed);

    return iteration_count;
  }

  std::array<Node<NodeCount, InfiniteDistance>, NodeCount> nodes_;

 private:
  void initNodes() {
    for (size_t id = 0; id < nodes_.size(); ++id) {
      nodes_[id].id_ = id;
      for (auto &node : nodes_) {
        node.initDvBasedOnDirectDv();
      }
    }
  }
};

}  // namespace dv

}  // end of namespace rco

#endif  // end of define DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP
