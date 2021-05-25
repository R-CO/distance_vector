/**
 * Author: R-CO
 * E-Mail: daniel1820kobe@gmail.com
 **/
#ifndef DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP
#define DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP

#include <algorithm>
#include <array>
#include <exception>
#include <limits>
#include <vector>
namespace rco {

namespace dv {

template <typename Ty, Ty INF>
class Distance {
 public:
  Distance() : val_(INF) {}
  explicit Distance(const Ty &v) : val_(v) {}

  auto &getValue() {
    return const_cast<Distance<Ty, INF> &>(
        static_cast<const Distance<Ty, INF> &>(*this).getValue());
  }

  auto &getValue() const { return this->val_; }

  auto &operator=(const Distance<Ty, INF> &obj) {
    this->val_ = obj.val_;
    return *this;
  }
  auto &operator=(const Ty &val) {
    val_ = val;
    return (*this);
  }

  inline auto operator==(const Distance<Ty, INF> &v) const {
    return this->val_ == v.val_;
  }

  inline auto operator!=(const Distance<Ty, INF> &v) const {
    return !(*this == v);
  }

  inline auto operator<(const Distance<Ty, INF> &v) const {
    if (this->val_ != INF && v.val_ == INF) {
      return true;
    } else if (this->val_ != INF && v.val_ != INF) {
      if (this->val_ < v.val_) {
        return true;
      }
    }

    return false;
  }

  inline auto operator>(const Distance<Ty, INF> &v) const { return v < *this; }

  inline auto operator<=(const Distance<Ty, INF> &v) const {
    return !(*this > v);
  }

  inline auto operator>=(const Distance<Ty, INF> &v) const {
    return !(v < *this);
  }

  inline auto operator+(const Distance<Ty, INF> &v) {
    if (this->val_ == INF || v.val_ == INF) {
      return Distance<Ty, INF>{INF};
    }

    return Distance<Ty, INF>{this->val_ + v.val_};
  }

  inline auto operator-(const Distance<Ty, INF> &v) {
    if (this->val_ == INF || v.val_ == INF) {
      return Distance<Ty, INF>{INF};
    }

    return Distance<Ty, INF>{this->val_ - v.val_};
  }

 private:
  Ty val_;
};

template <typename Ty, size_t NodeCount, Ty INF>
class DistanceVector {
 public:
  DistanceVector() {
    static thread_local Distance<Ty, INF> infinite{INF};
    distance_.fill(infinite);
  }

  explicit DistanceVector(std::array<Distance<Ty, INF>, NodeCount> &distance)
      : distance_(distance) {}

  void setDistanceVector(const std::vector<int> &vec) {
    if (vec.size() < NodeCount) {
      throw std::runtime_error("size of vec is less than NodeCount\n");
    }

    for (size_t i = 0; i < NodeCount; ++i) {
      distance_[i] = vec[i];
    }
  }

  void fillWithInfiniteDistance() {
    static thread_local Distance<Ty, INF> infinite{INF};
    distance_.fill(infinite);
  }

  auto begin() { return distance_.begin(); }
  auto begin() const { return distance_.begin(); }

  auto end() { return distance_.end(); }
  auto end() const { return distance_.end(); }

  auto &operator[](size_t idx) noexcept {
    return const_cast<Distance<Ty, INF> &>(
        static_cast<const DistanceVector<Ty, NodeCount, INF> &>(*this)[idx]);
  }
  const auto &operator[](size_t idx) const noexcept { return distance_[idx]; }

 private:
  std::array<Distance<Ty, INF>, NodeCount> distance_;
};

template <typename Ty, size_t NodeCount, Ty INF>
class Node {
 public:
  Node() : id_(std::numeric_limits<size_t>::max()) {}

  explicit Node(const DistanceVector<Ty, NodeCount, INF> &direct_distance)
      : id_(std::numeric_limits<size_t>::max()),
        direct_distance_(direct_distance) {}

  void setId(const size_t id) { this->id_ = id; }

  auto getId() const { return this->id_; }

  void setDirectDv(const std::vector<int> &vec) {
    direct_distance_.setDistanceVector(vec);
  }

  void initDvBasedOnDirectDv() {
    std::for_each(dv_of_neighbors_.begin(), dv_of_neighbors_.end(),
                  [](auto &distance_vector) {
                    distance_vector.fillWithInfiniteDistance();
                  });

    std::copy(direct_distance_.begin(), direct_distance_.end(),
              dv_of_neighbors_[id_].begin());
  }

  void computeDvOfMyself(bool &has_value_changed) {
    has_value_changed = false;
    for (size_t y = 0; y < NodeCount; ++y) {
      bool has_dx_y_changed = false;
      size_t x = id_;
      auto min_dx_y = dv_of_neighbors_[x][y];
      for (size_t v = 0; v < NodeCount; ++v) {
        auto computeDxvY = [this, y, v]() {
          static thread_local Distance<Ty, INF> infinite{INF};
          if (direct_distance_[v] == infinite ||
              dv_of_neighbors_[v][y] == infinite) {
            return infinite;
          } else {
            return direct_distance_[v] + dv_of_neighbors_[v][y];
          }
        };
        auto dx_v_y = computeDxvY();
        if (dx_v_y < min_dx_y) {
          min_dx_y = dx_v_y;
          has_dx_y_changed = true;
        }
      }

      if (has_dx_y_changed) {
        dv_of_neighbors_[x][y] = min_dx_y;
        has_value_changed = true;
      }
    }
  }

  void sendDvToNeighbors(
      std::array<Node<Ty, NodeCount, INF>, NodeCount> &neighbors) {
    for (auto &it : neighbors) {
      it.updateDvOfNodeX(this->id_, this->dv_of_neighbors_[id_]);
    }
  }

  void updateDvOfNodeX(const size_t id_x,
                       const DistanceVector<Ty, NodeCount, INF> &dv_of_x) {
    std::copy(dv_of_x.begin(), dv_of_x.end(), dv_of_neighbors_[id_x].begin());
  }

 private:
  size_t id_;

  DistanceVector<Ty, NodeCount, INF> direct_distance_;
  std::array<DistanceVector<Ty, NodeCount, INF>, NodeCount> dv_of_neighbors_;
};

template <typename Ty, size_t NodeCount, Ty INF>
class DistanceVectorSystem {
 public:
  // DistanceVectorSystem() { initNodes(); }

  explicit DistanceVectorSystem(
      std::array<Node<Ty, NodeCount, INF>, NodeCount> &nodes)
      : nodes_(nodes) {
    initNodes();
  }

  size_t run() {
    bool has_value_changed = false;
    size_t iteration_count = 0;
    do {
      has_value_changed = false;
      ++iteration_count;
      for (auto &node : nodes_) {
        bool node_has_value_changed = false;
        node.computeDvOfMyself(node_has_value_changed);
        if (node_has_value_changed) {
          node.sendDvToNeighbors(nodes_);
          has_value_changed = true;
        }
      }
    } while (has_value_changed);

    return iteration_count;
  }

  std::array<Node<Ty, NodeCount, INF>, NodeCount> nodes_;

 private:
  void initNodes() {
    for (size_t id = 0; id < nodes_.size(); ++id) {
      nodes_[id].setId(id);
      nodes_[id].initDvBasedOnDirectDv();
    }

    for (auto &node : nodes_) {
      node.sendDvToNeighbors(nodes_);
    }
  }
};  // end of class DistanceVectorSystem

}  // namespace dv

}  // end of namespace rco

#endif  // end of define DISTANCE_VECTOR_SRC_DISTANCE_VECTOR_HPP
