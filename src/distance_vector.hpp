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

  std::array<int, NodeCount> distance_;
};

template <size_t NodeCount, int InfiniteDistance>
class Node {
 public:
  Node() {}

  Node(DistanceVector<NodeCount, InfiniteDistance> &direct_distance)
      : direct_distance_(direct_distance) {}

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
    for (size_t y = 0; y < NodeCount; ++y) {
      auto &dx_y = dv_of_neighbors_[id_].distance_[y];
      auto dv_y = dv_of_v.distance_[y];
      auto cx_v = direct_distance_.distance_[id_v];
      if (dx_y == InfiniteDistance &&
          (cx_v == InfiniteDistance || dv_y == InfiniteDistance)) {
        continue;
      } else if (dx_y == InfiniteDistance) {
        dx_y = cx_v + dv_y;
        has_value_changed = true;
      } else if (dx_y != InfiniteDistance && cx_v != InfiniteDistance &&
                 dv_y != InfiniteDistance) {
        dx_y = std::min(cx_v + dv_y, dx_y);
        has_value_changed = true;
      }
    }
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
  DistanceVectorSystem() { initNodes(); }

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
      for (size_t neighbor_id = 0; neighbor_id < nodes_.size(); ++neighbor_id) {
        std::copy(nodes_[neighbor_id].direct_distance_.distance_.begin(),
                  nodes_[neighbor_id].direct_distance_.distance_.end(),
                  nodes_[id].dv_of_neighbors_[neighbor_id].distance_.begin());
        // nodes_[id].dv_of_neighbors_[neighbor_id].distance_ =
        //     nodes_[neighbor_id].direct_distance_;
        // nodes_[id].neighbors[neighbor_id] = &nodes_[neighbor_id]
      }
    }
  }
};

}  // namespace dv

}  // end of namespace rco

#endif  // end of define DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP
