#pragma once

#include <mapping_core/mapping.h>

#include <decomp_util/ellipsoid_decomp.h>
#include <traj_opt/geoutils.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace std {
template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
      Scalar elem = *(matrix.data() + i);
      seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
}  // namespace std

namespace env_ros2 {

enum State { OPEN, CLOSE, UNVISITED };

struct Node {
  Eigen::Vector3i idx;
  bool valid{false};
  State state{UNVISITED};
  double g{0.0}, h{0.0};
  Node* parent{nullptr};
};
using NodePtr = Node*;

class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) { return lhs->g + lhs->h > rhs->g + rhs->h; }
};

class Env {
  static constexpr int MAX_MEMORY = 1 << 18;
  static constexpr double MAX_DURATION = 0.2;

 private:
  using Clock = std::chrono::steady_clock;

  Clock::time_point t_start_{};
  std::unordered_map<Eigen::Vector3i, NodePtr> visited_nodes_;
  std::shared_ptr<mapping::OccGridMap> mapPtr_;
  NodePtr data_[MAX_MEMORY];

  double desired_dist_{2.0};
  double theta_clearance_{0.5};
  double tolerance_d_{0.5};

  inline void compressPoly(Polyhedron3D& poly, double dx) {
    vec_E<Hyperplane3D> hyper_planes = poly.hyperplanes();
    for (uint j = 0; j < hyper_planes.size(); j++) {
      hyper_planes[j].p_ = hyper_planes[j].p_ - hyper_planes[j].n_ * dx;
    }
    poly = Polyhedron3D(hyper_planes);
  }

  inline void compressPoly(Eigen::MatrixXd& poly, double dx) {
    for (int i = 0; i < poly.cols(); ++i) {
      poly.col(i).tail(3) = poly.col(i).tail(3) - poly.col(i).head(3) * dx;
    }
  }

  inline void getPointCloudAroundLine(const vec_Vec3f& line,
                                      const int maxWidth,
                                      vec_Vec3f& pc) {
    pc.clear();
    Eigen::Vector3d p0 = line.front();
    Eigen::Vector3d p1 = line.back();
    Eigen::Vector3i idx0 = mapPtr_->pos2idx(p0);
    Eigen::Vector3i idx1 = mapPtr_->pos2idx(p1);
    Eigen::Vector3i d_idx = idx1 - idx0;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d delta_t;
    Eigen::Vector3i tmp_p, margin;
    margin.setConstant(maxWidth);
    for (tmp_p.x() = idx0.x() - margin.x(); tmp_p.x() <= idx0.x() + margin.x(); ++tmp_p.x()) {
      for (tmp_p.y() = idx0.y() - margin.y(); tmp_p.y() <= idx0.y() + margin.y(); ++tmp_p.y()) {
        for (tmp_p.z() = idx0.z() - margin.z(); tmp_p.z() <= idx0.z() + margin.z(); ++tmp_p.z()) {
          if (mapPtr_->isOccupied(tmp_p)) pc.push_back(mapPtr_->idx2pos(tmp_p));
        }
      }
    }
    for (int i = 0; i < 3; ++i) {
      delta_t(i) = d_idx(i) == 0 ? 2.0 : 1.0 / std::abs(d_idx(i));
    }
    Eigen::Vector3d t_max;
    for (int i = 0; i < 3; ++i) {
      t_max(i) = step(i) > 0 ? std::ceil(p0(i)) - p0(i) : p0(i) - std::floor(p0(i));
    }
    t_max = t_max.cwiseProduct(delta_t);
    Eigen::Vector3i rayIdx = idx0;
    while (rayIdx != idx1) {
      int s_dim = 0;
      for (int i = 1; i < 3; ++i) s_dim = t_max(i) < t_max(s_dim) ? i : s_dim;
      rayIdx(s_dim) += step(s_dim);
      t_max(s_dim) += delta_t(s_dim);
      margin.setConstant(maxWidth);
      margin(s_dim) = 0;
      Eigen::Vector3i center = rayIdx;
      center(s_dim) += maxWidth * step(s_dim);
      for (tmp_p.x() = center.x() - margin.x(); tmp_p.x() <= center.x() + margin.x(); ++tmp_p.x()) {
        for (tmp_p.y() = center.y() - margin.y(); tmp_p.y() <= center.y() + margin.y(); ++tmp_p.y()) {
          for (tmp_p.z() = center.z() - margin.z(); tmp_p.z() <= center.z() + margin.z(); ++tmp_p.z()) {
            if (mapPtr_->isOccupied(tmp_p)) pc.push_back(mapPtr_->idx2pos(tmp_p));
          }
        }
      }
    }
  }

  inline NodePtr visit(const Eigen::Vector3i& idx) {
    auto iter = visited_nodes_.find(idx);
    if (iter == visited_nodes_.end()) {
      auto ptr = data_[visited_nodes_.size()];
      ptr->idx = idx;
      ptr->valid = !mapPtr_->isOccupied(idx);
      ptr->state = UNVISITED;
      visited_nodes_[idx] = ptr;
      return ptr;
    } else {
      return iter->second;
    }
  }

  inline double elapsedSec() const {
    return std::chrono::duration<double>(Clock::now() - t_start_).count();
  }

 public:
  Env(const std::shared_ptr<mapping::OccGridMap>& mapPtr,
      double desired_dist,
      double tolerance_d,
      double theta_clearance)
  : mapPtr_(mapPtr), desired_dist_(desired_dist), theta_clearance_(theta_clearance), tolerance_d_(tolerance_d) {
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data_[i] = new Node;
    }
  }

  ~Env() {
    for (int i = 0; i < MAX_MEMORY; ++i) {
      delete data_[i];
    }
  }

  bool inline checkRayValid(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double max_dist) const {
    Eigen::Vector3d dp = p1 - p0;
    double dist = dp.norm();
    if (dist > max_dist) return false;

    Eigen::Vector3i idx0 = mapPtr_->pos2idx(p0);
    Eigen::Vector3i idx1 = mapPtr_->pos2idx(p1);
    Eigen::Vector3i d_idx = idx1 - idx0;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d delta_t;
    for (int i = 0; i < 3; ++i) {
      delta_t(i) = dp(i) == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp(i));
    }
    Eigen::Vector3d t_max;
    for (int i = 0; i < 3; ++i) {
      t_max(i) = step(i) > 0 ? (idx0(i) + 1) - p0(i) / mapPtr_->resolution : p0(i) / mapPtr_->resolution - idx0(i);
    }
    t_max = t_max.cwiseProduct(delta_t);
    Eigen::Vector3i rayIdx = idx0;
    while ((rayIdx - idx1).squaredNorm() > 1) {
      if (mapPtr_->isOccupied(rayIdx)) return false;
      int s_dim = 0;
      for (int i = 1; i < 3; ++i) s_dim = t_max(i) < t_max(s_dim) ? i : s_dim;
      rayIdx(s_dim) += step(s_dim);
      t_max(s_dim) += delta_t(s_dim);
    }
    return true;
  }

  bool inline checkRayValid(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) const {
    Eigen::Vector3d dp = p1 - p0;
    Eigen::Vector3i idx0 = mapPtr_->pos2idx(p0);
    Eigen::Vector3i idx1 = mapPtr_->pos2idx(p1);
    Eigen::Vector3i d_idx = idx1 - idx0;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d delta_t;
    for (int i = 0; i < 3; ++i) {
      delta_t(i) = dp(i) == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp(i));
    }
    Eigen::Vector3d t_max;
    for (int i = 0; i < 3; ++i) {
      t_max(i) = step(i) > 0 ? (idx0(i) + 1) - p0(i) / mapPtr_->resolution : p0(i) / mapPtr_->resolution - idx0(i);
    }
    t_max = t_max.cwiseProduct(delta_t);
    Eigen::Vector3i rayIdx = idx0;
    while ((rayIdx - idx1).squaredNorm() > 1) {
      if (mapPtr_->isOccupied(rayIdx)) return false;
      int s_dim = 0;
      for (int i = 1; i < 3; ++i) s_dim = t_max(i) < t_max(s_dim) ? i : s_dim;
      rayIdx(s_dim) += step(s_dim);
      t_max(s_dim) += delta_t(s_dim);
    }
    return true;
  }

  bool rayValid(const Eigen::Vector3i& idx0, const Eigen::Vector3i& idx1) {
    Eigen::Vector3i d_idx = idx1 - idx0;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d delta_t;
    for (int i = 0; i < 3; ++i) {
      delta_t(i) = d_idx(i) == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(d_idx(i));
    }
    Eigen::Vector3d t_max(0.5, 0.5, 0.5);
    t_max = t_max.cwiseProduct(delta_t);
    Eigen::Vector3i rayIdx = idx0;
    while ((rayIdx - idx1).squaredNorm() > 1) {
      if (mapPtr_->isOccupied(rayIdx)) return false;
      int s_dim = 0;
      for (int i = 1; i < 3; ++i) s_dim = t_max(i) < t_max(s_dim) ? i : s_dim;
      rayIdx(s_dim) += step(s_dim);
      t_max(s_dim) += delta_t(s_dim);
    }
    return true;
  }

  inline bool astar_search(const Eigen::Vector3i& start_idx,
                           const Eigen::Vector3i& end_idx,
                           std::vector<Eigen::Vector3i>& idx_path) {
    auto stopCondition = [&](const NodePtr& ptr) -> bool { return ptr->h < desired_dist_ / mapPtr_->resolution; };
    auto calulateHeuristic = [&](const NodePtr& ptr) {
      Eigen::Vector3i dp = end_idx - ptr->idx;
      int dx = dp.x(), dy = dp.y(), dz = dp.z();
      ptr->h = abs(dx) + abs(dy) + abs(dz);
      double dx0 = (start_idx - end_idx).x();
      double dy0 = (start_idx - end_idx).y();
      double cross = fabs(dx * dy0 - dy * dx0) + abs(dz);
      ptr->h += 0.001 * cross;
    };

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;
    std::vector<std::pair<Eigen::Vector3i, double>> neighbors;
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3i neighbor(0, 0, 0);
      neighbor[i] = 1;
      neighbors.emplace_back(neighbor, 1);
      neighbor[i] = -1;
      neighbors.emplace_back(neighbor, 1);
    }

    bool ret = false;
    NodePtr curPtr = visit(start_idx);
    if (!curPtr->valid) {
      visited_nodes_.clear();
      return false;
    }
    curPtr->parent = nullptr;
    curPtr->g = 0;
    calulateHeuristic(curPtr);
    curPtr->state = CLOSE;

    while (visited_nodes_.size() < MAX_MEMORY) {
      for (const auto& neighbor : neighbors) {
        const auto neighbor_idx = curPtr->idx + neighbor.first;
        const auto neighbor_dist = neighbor.second;
        NodePtr neighborPtr = visit(neighbor_idx);
        if (neighborPtr->state == CLOSE) continue;
        if (neighborPtr->state == OPEN) {
          if (neighborPtr->g > curPtr->g + neighbor_dist) {
            neighborPtr->parent = curPtr;
            neighborPtr->g = curPtr->g + neighbor_dist;
          }
          continue;
        }
        if (neighborPtr->state == UNVISITED) {
          if (neighborPtr->valid) {
            neighborPtr->parent = curPtr;
            neighborPtr->state = OPEN;
            neighborPtr->g = curPtr->g + neighbor_dist;
            calulateHeuristic(neighborPtr);
            open_set.push(neighborPtr);
          }
        }
      }
      if (open_set.empty()) break;
      curPtr = open_set.top();
      open_set.pop();
      curPtr->state = CLOSE;
      if (stopCondition(curPtr)) {
        ret = true;
        break;
      }
    }

    if (ret) {
      for (NodePtr ptr = curPtr; ptr != nullptr; ptr = ptr->parent) {
        idx_path.push_back(ptr->idx);
      }
      std::reverse(idx_path.begin(), idx_path.end());
    }
    visited_nodes_.clear();
    return ret;
  }

  inline bool astar_search(const Eigen::Vector3d& start_p,
                           const Eigen::Vector3d& end_p,
                           std::vector<Eigen::Vector3d>& path) {
    Eigen::Vector3i start_idx = mapPtr_->pos2idx(start_p);
    Eigen::Vector3i end_idx = mapPtr_->pos2idx(end_p);
    std::vector<Eigen::Vector3i> idx_path;
    const bool ret = astar_search(start_idx, end_idx, idx_path);
    path.clear();
    for (const auto& id : idx_path) path.push_back(mapPtr_->idx2pos(id));
    return ret;
  }

  inline bool short_astar(const Eigen::Vector3d& start_p,
                          const Eigen::Vector3d& end_p,
                          std::vector<Eigen::Vector3d>& path) {
    Eigen::Vector3i start_idx = mapPtr_->pos2idx(start_p);
    Eigen::Vector3i end_idx = mapPtr_->pos2idx(end_p);
    if (start_idx == end_idx) {
      path.clear();
      path.push_back(start_p);
      path.push_back(end_p);
      return true;
    }

    auto stopCondition = [&](const NodePtr& ptr) -> bool { return ptr->idx == end_idx; };
    auto calulateHeuristic = [&](const NodePtr& ptr) {
      Eigen::Vector3i dp = end_idx - ptr->idx;
      int dx = dp.x(), dy = dp.y(), dz = dp.z();
      ptr->h = abs(dx) + abs(dy) + abs(dz);
      double dx0 = (start_idx - end_idx).x();
      double dy0 = (start_idx - end_idx).y();
      double cross = fabs(dx * dy0 - dy * dx0) + abs(dz);
      ptr->h += 0.001 * cross;
    };

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;
    std::vector<std::pair<Eigen::Vector3i, double>> neighbors;
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3i neighbor(0, 0, 0);
      neighbor[i] = 1;
      neighbors.emplace_back(neighbor, 1);
      neighbor[i] = -1;
      neighbors.emplace_back(neighbor, 1);
    }

    bool ret = false;
    NodePtr curPtr = visit(start_idx);
    if (!curPtr->valid) {
      visited_nodes_.clear();
      return false;
    }
    curPtr->parent = nullptr;
    curPtr->g = 0;
    calulateHeuristic(curPtr);
    curPtr->state = CLOSE;

    while (visited_nodes_.size() < MAX_MEMORY) {
      for (const auto& neighbor : neighbors) {
        const auto neighbor_idx = curPtr->idx + neighbor.first;
        const auto neighbor_dist = neighbor.second;
        NodePtr neighborPtr = visit(neighbor_idx);
        if (neighborPtr->state == CLOSE) continue;
        if (neighborPtr->state == OPEN) {
          if (neighborPtr->g > curPtr->g + neighbor_dist) {
            neighborPtr->parent = curPtr;
            neighborPtr->g = curPtr->g + neighbor_dist;
          }
          continue;
        }
        if (neighborPtr->state == UNVISITED) {
          if (neighborPtr->valid) {
            neighborPtr->parent = curPtr;
            neighborPtr->state = OPEN;
            neighborPtr->g = curPtr->g + neighbor_dist;
            calulateHeuristic(neighborPtr);
            open_set.push(neighborPtr);
          }
        }
      }
      if (open_set.empty()) break;
      curPtr = open_set.top();
      open_set.pop();
      curPtr->state = CLOSE;
      if (stopCondition(curPtr)) {
        ret = true;
        break;
      }
    }

    if (ret) {
      // follow ROS1 behavior: skip endpoints
      for (NodePtr ptr = curPtr->parent; ptr && ptr->parent != nullptr; ptr = ptr->parent) {
        path.push_back(mapPtr_->idx2pos(ptr->idx));
      }
      std::reverse(path.begin(), path.end());
    }
    visited_nodes_.clear();
    return ret;
  }

  inline void pts2path(const std::vector<Eigen::Vector3d>& wayPts, std::vector<Eigen::Vector3d>& path) {
    path.clear();
    if (wayPts.empty()) return;
    path.push_back(wayPts.front());
    const int M = static_cast<int>(wayPts.size());
    std::vector<Eigen::Vector3d> short_path;
    for (int i = 0; i < M - 1; ++i) {
      const Eigen::Vector3d& p0 = path.back();
      const Eigen::Vector3d& p1 = wayPts[i + 1];
      if (mapPtr_->pos2idx(p0) == mapPtr_->pos2idx(p1)) continue;
      if (!checkRayValid(p0, p1, 1.5)) {
        short_path.clear();
        (void)short_astar(p0, p1, short_path);
        for (const auto& p : short_path) path.push_back(p);
      }
      path.push_back(p1);
    }
    if (path.size() < 2) {
      Eigen::Vector3d p = path.front();
      p.z() += 0.1;
      path.push_back(p);
    }
  }

  // Visible region generation (ported from ROS1 Env)
  inline void visible_pair(const Eigen::Vector3d& center,
                           Eigen::Vector3d& seed,
                           Eigen::Vector3d& visible_p,
                           double& theta) const {
    Eigen::Vector3d dp = seed - center;
    double theta0 = atan2(dp.y(), dp.x());
    double d_theta = mapPtr_->resolution / desired_dist_ / 2;
    double t_l, t_r;
    for (t_l = theta0 - d_theta; t_l > theta0 - M_PI; t_l -= d_theta) {
      Eigen::Vector3d p = center;
      p.x() += desired_dist_ * cos(t_l);
      p.y() += desired_dist_ * sin(t_l);
      if (!checkRayValid(p, center)) {
        t_l += d_theta;
        break;
      }
    }
    for (t_r = theta0 + d_theta; t_r < theta0 + M_PI; t_r += d_theta) {
      Eigen::Vector3d p = center;
      p.x() += desired_dist_ * cos(t_r);
      p.y() += desired_dist_ * sin(t_r);
      if (!checkRayValid(p, center)) {
        t_r -= d_theta;
        break;
      }
    }
    double theta_v = (t_l + t_r) / 2;
    visible_p = center;
    visible_p.x() += desired_dist_ * cos(theta_v);
    visible_p.y() += desired_dist_ * sin(theta_v);
    theta = (t_r - t_l) / 2;
    double theta_c = theta < theta_clearance_ ? theta : theta_clearance_;
    if (theta0 - t_l < theta_c) {
      seed = center;
      seed.x() += desired_dist_ * cos(t_l + theta_c);
      seed.y() += desired_dist_ * sin(t_l + theta_c);
    } else if (t_r - theta0 < theta_c) {
      seed = center;
      seed.x() += desired_dist_ * cos(t_r - theta_c);
      seed.y() += desired_dist_ * sin(t_r - theta_c);
    }
  }

  inline void generate_visible_regions(const std::vector<Eigen::Vector3d>& targets,
                                       std::vector<Eigen::Vector3d>& seeds,
                                       std::vector<Eigen::Vector3d>& visible_ps,
                                       std::vector<double>& thetas) const {
    visible_ps.clear();
    thetas.clear();
    if (targets.size() != seeds.size()) return;
    Eigen::Vector3d visible_p;
    double theta = 0;
    const int M = static_cast<int>(targets.size());
    for (int i = 0; i < M; ++i) {
      visible_pair(targets[i], seeds[i], visible_p, theta);
      visible_ps.push_back(visible_p);
      thetas.push_back(theta);
    }
  }

  // Corridor generation (SFC) using Decomp (ported from ROS1 Env)
  inline bool filterCorridor(std::vector<Eigen::MatrixXd>& hPolys) const {
    bool ret = false;
    if (hPolys.size() <= 2) return ret;
    std::vector<Eigen::MatrixXd> ret_polys;
    Eigen::MatrixXd hPoly0 = hPolys[0];
    Eigen::MatrixXd curIH;
    Eigen::Vector3d interior;
    for (int i = 2; i < static_cast<int>(hPolys.size()); i++) {
      curIH.resize(6, hPoly0.cols() + hPolys[i].cols());
      curIH << hPoly0, hPolys[i];
      if (geoutils::findInteriorDist(curIH, interior) < 1.0) {
        ret_polys.push_back(hPoly0);
        hPoly0 = hPolys[i - 1];
      } else {
        ret = true;
      }
    }
    ret_polys.push_back(hPoly0);
    ret_polys.push_back(hPolys.back());
    hPolys = ret_polys;
    return ret;
  }

  inline void generateSFC(const std::vector<Eigen::Vector3d>& path,
                          const double bbox_width,
                          std::vector<Eigen::MatrixXd>& hPolys,
                          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& keyPts) {
    if (path.size() < 2) {
      hPolys.clear();
      keyPts.clear();
      return;
    }
    vec_Vec3f obs_pc;
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_local_bbox(Eigen::Vector3d(bbox_width, bbox_width, bbox_width));
    const int maxWidth = static_cast<int>(bbox_width / mapPtr_->resolution);
    vec_E<Polyhedron3D> decompPolys;
    const int path_len = static_cast<int>(path.size());
    int idx = 0;
    keyPts.clear();
    while (idx < path_len - 1) {
      int next_idx = idx;
      while (next_idx + 1 < path_len && checkRayValid(path[idx], path[next_idx + 1], bbox_width)) {
        next_idx++;
      }
      vec_Vec3f line;
      line.push_back(path[idx]);
      line.push_back(path[next_idx]);
      keyPts.emplace_back(path[idx], path[next_idx]);
      getPointCloudAroundLine(line, maxWidth, obs_pc);
      decomp_util.set_obs(obs_pc);
      decomp_util.dilate(line);
      Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
      decompPolys.push_back(poly);
      idx = next_idx;
      while (idx + 1 < path_len && decompPolys.back().inside(path[idx + 1])) {
        idx++;
      }
    }
    hPolys.clear();
    Eigen::MatrixXd current_poly;
    for (uint i = 0; i < decompPolys.size(); i++) {
      vec_E<Hyperplane3D> current_hyperplanes = decompPolys[i].hyperplanes();
      current_poly.resize(6, current_hyperplanes.size());
      for (uint j = 0; j < current_hyperplanes.size(); j++) {
        current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
      }
      hPolys.push_back(current_poly);
    }
    (void)filterCorridor(hPolys);
    Eigen::Vector3d interior;
    std::vector<int> inflate(hPolys.size(), 0);
    for (int i = 0; i < static_cast<int>(hPolys.size()); i++) {
      if (geoutils::findInteriorDist(current_poly, interior) < 0.1) {
        inflate[i] = 1;
      } else {
        compressPoly(hPolys[i], 0.1);
      }
    }
    Eigen::MatrixXd curIH;
    for (int i = 1; i < static_cast<int>(hPolys.size()); i++) {
      curIH.resize(6, hPolys[i - 1].cols() + hPolys[i].cols());
      curIH << hPolys[i - 1], hPolys[i];
      if (!geoutils::findInterior(curIH, interior)) {
        if (!inflate[i - 1]) {
          compressPoly(hPolys[i - 1], -0.1);
          inflate[i - 1] = 1;
        }
      } else {
        continue;
      }
      curIH << hPolys[i - 1], hPolys[i];
      if (!geoutils::findInterior(curIH, interior)) {
        if (!inflate[i]) {
          compressPoly(hPolys[i], -0.1);
          inflate[i] = 1;
        }
      }
    }
  }

  // Visible-path search (ported from ROS1)
  inline bool findVisiblePath(const Eigen::Vector3d& start_p,
                              const std::vector<Eigen::Vector3d>& targets,
                              std::vector<Eigen::Vector3d>& way_pts,
                              std::vector<Eigen::Vector3d>& path) {
    t_start_ = Clock::now();
    Eigen::Vector3i start_idx = mapPtr_->pos2idx(start_p);
    std::vector<Eigen::Vector3i> idx_path;
    path.clear();
    way_pts.clear();
    path.push_back(start_p);
    for (const auto& target : targets) {
      Eigen::Vector3i end_idx = mapPtr_->pos2idx(target);
      idx_path.clear();
      if (!findVisiblePath(start_idx, end_idx, idx_path)) return false;
      start_idx = idx_path.back();
      for (const auto& idx : idx_path) path.push_back(mapPtr_->idx2pos(idx));
      way_pts.push_back(mapPtr_->idx2pos(start_idx));
    }
    return true;
  }

  inline bool findVisiblePath(const Eigen::Vector3i& start_idx,
                              const Eigen::Vector3i& end_idx,
                              std::vector<Eigen::Vector3i>& idx_path) {
    const double stop_dist = desired_dist_ / mapPtr_->resolution;
    auto stopCondition = [&](const NodePtr& ptr) -> bool {
      return ptr->h < tolerance_d_ / mapPtr_->resolution && rayValid(ptr->idx, end_idx);
    };
    auto calulateHeuristic = [&](const NodePtr& ptr) {
      Eigen::Vector3i dp = end_idx - ptr->idx;
      const double dr = dp.head(2).norm();
      const double lambda = 1 - stop_dist / dr;
      const double dx = lambda * dp.x();
      const double dy = lambda * dp.y();
      const double dz = dp.z();
      ptr->h = fabs(dx) + fabs(dy) + abs(dz);
      const double dx0 = (start_idx - end_idx).x();
      const double dy0 = (start_idx - end_idx).y();
      const double cross = fabs(dx * dy0 - dy * dx0) + abs(dz);
      ptr->h += 0.001 * cross;
    };

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;
    std::vector<std::pair<Eigen::Vector3i, double>> neighbors;
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3i neighbor(0, 0, 0);
      neighbor[i] = 1;
      neighbors.emplace_back(neighbor, 1);
      neighbor[i] = -1;
      neighbors.emplace_back(neighbor, 1);
    }

    bool ret = false;
    NodePtr curPtr = visit(start_idx);
    if (!curPtr->valid) {
      visited_nodes_.clear();
      return false;
    }
    curPtr->parent = nullptr;
    curPtr->g = 0;
    calulateHeuristic(curPtr);
    curPtr->state = CLOSE;

    double t_cost = elapsedSec();
    while (visited_nodes_.size() < MAX_MEMORY && t_cost <= MAX_DURATION) {
      for (const auto& neighbor : neighbors) {
        const auto neighbor_idx = curPtr->idx + neighbor.first;
        const auto neighbor_dist = neighbor.second;
        NodePtr neighborPtr = visit(neighbor_idx);
        if (neighborPtr->state == CLOSE) continue;
        if (neighborPtr->state == OPEN) {
          if (neighborPtr->g > curPtr->g + neighbor_dist) {
            neighborPtr->parent = curPtr;
            neighborPtr->g = curPtr->g + neighbor_dist;
          }
          continue;
        }
        if (neighborPtr->state == UNVISITED) {
          if (neighborPtr->valid) {
            neighborPtr->parent = curPtr;
            neighborPtr->state = OPEN;
            neighborPtr->g = curPtr->g + neighbor_dist;
            calulateHeuristic(neighborPtr);
            open_set.push(neighborPtr);
          }
        }
      }
      if (open_set.empty()) break;
      curPtr = open_set.top();
      open_set.pop();
      curPtr->state = CLOSE;
      if (stopCondition(curPtr)) {
        ret = true;
        break;
      }
      t_cost = elapsedSec();
    }

    if (ret) {
      for (NodePtr ptr = curPtr; ptr != nullptr; ptr = ptr->parent) {
        idx_path.push_back(ptr->idx);
      }
      std::reverse(idx_path.begin(), idx_path.end());
    }
    visited_nodes_.clear();
    return ret;
  }
};

}  // namespace env_ros2

