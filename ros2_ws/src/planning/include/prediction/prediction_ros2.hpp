#pragma once

#include <mapping_core/mapping.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>

namespace prediction_ros2 {

struct Node {
  Eigen::Vector3d p, v, a;
  double t{0.0};
  double score{0.0};
  double h{0.0};
  Node* parent{nullptr};
};
using NodePtr = Node*;

class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) { return lhs->score + lhs->h > rhs->score + rhs->h; }
};

class Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;

  double dt_{0.1};
  double pre_dur_{1.0};
  double rho_a_{1.0};
  double vmax_{5.0};

  mapping::OccGridMap map_;
  NodePtr data_[MAX_MEMORY];
  int stack_top_{0};

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return (v.norm() < vmax_) && (!map_.isOccupied(p));
  }

 public:
  Predict(double pre_dur, double dt, double rho_a, double vmax) : dt_(dt), pre_dur_(pre_dur), rho_a_(rho_a), vmax_(vmax) {
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data_[i] = new Node;
    }
  }

  ~Predict() {
    for (int i = 0; i < MAX_MEMORY; ++i) {
      delete data_[i];
    }
  }

  inline void setMap(const mapping::OccGridMap& map) { map_ = map; }

  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predict,
                      const double max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double { return rho_a_ * ptr->a.norm(); };
    const Eigen::Vector3d end_p = target_p + target_v * pre_dur_;
    auto calH = [&](const NodePtr& ptr) -> double { return 0.001 * (ptr->p - end_p).norm(); };

    const auto t_start = std::chrono::steady_clock::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);
    stack_top_ = 0;
    NodePtr curPtr = data_[stack_top_++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;

    const double dt2_2 = dt_ * dt_ / 2;
    while (curPtr->t < pre_dur_) {
      for (input.x() = -3; input.x() <= 3; input.x() += 3) {
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          const Eigen::Vector3d p = curPtr->p + curPtr->v * dt_ + input * dt2_2;
          const Eigen::Vector3d v = curPtr->v + input * dt_;
          if (!isValid(p, v)) continue;
          if (stack_top_ == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          const double t_cost = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          NodePtr ptr = data_[stack_top_++];
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt_;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
        }
      }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }

    target_predict.clear();
    while (curPtr != nullptr) {
      target_predict.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predict.begin(), target_predict.end());
    return true;
  }
};

}  // namespace prediction_ros2

