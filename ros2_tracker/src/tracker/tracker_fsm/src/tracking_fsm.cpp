#include "tracker_fsm/tracking_fsm.h"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <cmath>
#include <traj_opt/poly_traj_utils.hpp>

namespace tracker {

void TrackingFSM::init(rclcpp::Node::SharedPtr node) {
  node_ = node;
  
  // Declare and get parameters
  node_->declare_parameter("plan_hz", 10);
  node_->declare_parameter("tracking_dur", 0.0);
  node_->declare_parameter("tracking_dist", 0.0);
  node_->declare_parameter("tolerance_d", 0.0);
  node_->declare_parameter("debug", false);
  node_->declare_parameter("fake", false);
  
  node_->get_parameter("plan_hz", plan_hz_);
  node_->get_parameter("tracking_dur", tracking_dur_);
  node_->get_parameter("tracking_dist", tracking_dist_);
  node_->get_parameter("tolerance_d", tolerance_d_);
  node_->get_parameter("debug", debug_);
  node_->get_parameter("fake", fake_);
  
  // Initialize sub-modules
  mapper_.init(node);
  // Use mapper's internal gridmap via shared pointer
  // Note: mapper maintains its own gridmap, we'll use it via reference
  gridmap_ptr_ = std::make_shared<mapping::OccGridMap>();
  // Only initialize EKF in tracking mode (not fake/navigation mode)
  if (!fake_) {
    target_ekf_.init(node);
  }
  env_ptr_ = std::make_shared<env::Env>(node, gridmap_ptr_);
  vis_ptr_ = std::make_shared<visualization::Visualization>(node);
  traj_opt_ptr_ = std::make_shared<traj_opt::TrajOpt>(node);
  pre_ptr_ = std::make_shared<prediction::Predict>(node);
  
  // Create publishers
  heartbeat_pub_ = node_->create_publisher<std_msgs::msg::Empty>("heartbeat", 10);
  traj_pub_ = node_->create_publisher<quadrotor_msgs::msg::PolyTraj>("trajectory", 1);
  replan_state_pub_ = node_->create_publisher<quadrotor_msgs::msg::ReplanState>("replanState", 1);
  
  // Create subscribers
  gridmap_sub_ = node_->create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap_inflate", 1,
      std::bind(&TrackingFSM::gridmapCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TrackingFSM::odomCallback, this, std::placeholders::_1));
  target_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "target", 10,
      std::bind(&TrackingFSM::targetCallback, this, std::placeholders::_1));
  triger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "triger", 10,
      std::bind(&TrackingFSM::trigerCallback, this, std::placeholders::_1));
  land_triger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "land_triger", 10,
      std::bind(&TrackingFSM::landTrigerCallback, this, std::placeholders::_1));
  
  // Initialize replan_stamp_ with ROS_TIME source to avoid
  // "can't subtract times with different time sources" crash
  replan_stamp_ = node_->now();
  force_hover_ = true;  // Start in hover mode until first plan
  
  // Create timer based on mode
  if (fake_) {
    plan_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000 / plan_hz_),
        std::bind(&TrackingFSM::fakeTimerCallback, this));
  } else {
    plan_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000 / plan_hz_),
        std::bind(&TrackingFSM::planTimerCallback, this));
  }
  
  RCLCPP_WARN(node_->get_logger(), "Tracking FSM initialized!");
}

void TrackingFSM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  while (odom_lock_.test_and_set())
    ;
  odom_msg_ = *msg;
  odom_received_ = true;
  odom_lock_.clear();
}

void TrackingFSM::targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  while (target_lock_.test_and_set())
    ;
  target_msg_ = *msg;
  target_received_ = true;
  target_lock_.clear();
}

void TrackingFSM::gridmapCallback(const quadrotor_msgs::msg::OccMap3d::SharedPtr msg) {
  while (gridmap_lock_.test_and_set())
    ;
  map_msg_ = *msg;
  map_received_ = true;
  gridmap_lock_.clear();
}

void TrackingFSM::trigerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_ << msg->pose.position.x, msg->pose.position.y, 0.9;
  triger_received_ = true;
}

void TrackingFSM::landTrigerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  land_p_.x() = msg->pose.position.x;
  land_p_.y() = msg->pose.position.y;
  land_p_.z() = msg->pose.position.z;
  land_q_.w() = msg->pose.orientation.w;
  land_q_.x() = msg->pose.orientation.x;
  land_q_.y() = msg->pose.orientation.y;
  land_q_.z() = msg->pose.orientation.z;
  land_triger_received_ = true;
}

void TrackingFSM::pubHoverP(const Eigen::Vector3d& hover_p, const rclcpp::Time& stamp) {
  quadrotor_msgs::msg::PolyTraj traj_msg;
  traj_msg.hover = true;
  traj_msg.hover_p.resize(3);
  for (int i = 0; i < 3; ++i) {
    traj_msg.hover_p[i] = hover_p[i];
  }
  traj_msg.start_time = stamp;
  traj_msg.traj_id = traj_id_++;
  traj_pub_->publish(traj_msg);
}

void TrackingFSM::pubTraj(const Trajectory& traj, double yaw, const rclcpp::Time& stamp) {
  quadrotor_msgs::msg::PolyTraj msg;
  msg.hover = false;
  msg.drone_id = 0;
  msg.traj_id = traj_id_++;
  msg.start_time = stamp;
  msg.order = 5;
  msg.yaw = yaw;
  
  Eigen::VectorXd dur = traj.getDurations();
  int piece_num = traj.getPieceNum();
  msg.duration.resize(piece_num);
  msg.coef_x.resize(6 * piece_num);
  msg.coef_y.resize(6 * piece_num);
  msg.coef_z.resize(6 * piece_num);
  
  for (int i = 0; i < piece_num; ++i) {
    msg.duration[i] = dur(i);
    CoefficientMat cMat = traj[i].getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++) {
      msg.coef_x[i6 + j] = cMat(0, j);
      msg.coef_y[i6 + j] = cMat(1, j);
      msg.coef_z[i6 + j] = cMat(2, j);
    }
  }
  traj_pub_->publish(msg);
}

bool TrackingFSM::validcheck(const Trajectory& traj, const rclcpp::Time& t_start, double check_dur) {
  double t0 = (node_->now() - t_start).seconds();
  t0 = t0 > 0.0 ? t0 : 0.0;
  double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
  for (double t = t0; t < t0 + delta_t; t += 0.01) {
    Eigen::Vector3d p = traj.getPos(t);
    if (gridmap_ptr_->isOccupied(p)) {
      return false;
    }
  }
  return true;
}

void TrackingFSM::planTimerCallback() {
  heartbeat_pub_->publish(std_msgs::msg::Empty());
  if (!odom_received_ || !map_received_) {
    return;
  }
  
  // Obtain state of odom
  while (odom_lock_.test_and_set())
    ;
  auto odom_msg = odom_msg_;
  odom_lock_.clear();
  Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                         odom_msg.pose.pose.position.y,
                         odom_msg.pose.pose.position.z);
  Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                         odom_msg.twist.twist.linear.y,
                         odom_msg.twist.twist.linear.z);
  Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                            odom_msg.pose.pose.orientation.x,
                            odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z);
  
  if (!triger_received_) {
    return;
  }
  if (!target_received_) {
    return;
  }
  
  // Obtain state of target
  while (target_lock_.test_and_set())
    ;
  replan_state_msg_.target = target_msg_;
  target_lock_.clear();
  Eigen::Vector3d target_p(replan_state_msg_.target.pose.pose.position.x,
                           replan_state_msg_.target.pose.pose.position.y,
                           replan_state_msg_.target.pose.pose.position.z);
  Eigen::Vector3d target_v(replan_state_msg_.target.twist.twist.linear.x,
                           replan_state_msg_.target.twist.twist.linear.y,
                           replan_state_msg_.target.twist.twist.linear.z);
  Eigen::Quaterniond target_q;
  target_q.w() = replan_state_msg_.target.pose.pose.orientation.w;
  target_q.x() = replan_state_msg_.target.pose.pose.orientation.x;
  target_q.y() = replan_state_msg_.target.pose.pose.orientation.y;
  target_q.z() = replan_state_msg_.target.pose.pose.orientation.z;
  
  // Force-hover: waiting for the speed of drone small enough
  if (force_hover_ && odom_v.norm() > 0.1) {
    return;
  }
  
  // Just for landing on the car!
  if (land_triger_received_) {
    if (std::fabs((target_p - odom_p).norm()) < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2) {
      if (!wait_hover_) {
        pubHoverP(odom_p, node_->now());
        wait_hover_ = true;
      }
      RCLCPP_WARN(node_->get_logger(), "[planner] HOVERING...");
      return;
    }
    target_p = target_p + target_q * land_p_;
    wait_hover_ = false;
  } else {
    target_p.z() += 1.0;
    // Determine whether to replan
    Eigen::Vector3d dp = target_p - odom_p;
    double desired_yaw = std::atan2(dp.y(), dp.x());
    Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
    double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
    if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ &&
        odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
        std::fabs(desired_yaw - now_yaw) < 0.5) {
      if (!wait_hover_) {
        pubHoverP(odom_p, node_->now());
        wait_hover_ = true;
      }
      RCLCPP_WARN(node_->get_logger(), "[planner] HOVERING...");
      replan_state_msg_.state = -1;
      replan_state_pub_->publish(replan_state_msg_);
      return;
    } else {
      wait_hover_ = false;
    }
  }
  
  // Obtain map
  while (gridmap_lock_.test_and_set())
    ;
  gridmap_ptr_->from_msg(map_msg_);
  replan_state_msg_.occmap = map_msg_;
  gridmap_lock_.clear();
  pre_ptr_->setMap(*gridmap_ptr_);
  
  // Visualize the ray from drone to target
  if (env_ptr_->checkRayValid(odom_p, target_p)) {
    vis_ptr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
  } else {
    vis_ptr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
  }
  
  // Prediction
  std::vector<Eigen::Vector3d> target_predict;
  bool generate_new_traj_success = pre_ptr_->predict(target_p, target_v, target_predict);
  if (generate_new_traj_success) {
    Eigen::Vector3d observable_p = target_predict.back();
    vis_ptr_->visualize_path(target_predict, "car_predict");
    std::vector<Eigen::Vector3d> observable_margin;
    for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
      observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
    }
    vis_ptr_->visualize_path(observable_margin, "observable_margin");
  }
  
  // Replan state
  Eigen::MatrixXd iniState;
  iniState.setZero(3, 3);
  rclcpp::Time replan_stamp = node_->now() + rclcpp::Duration(0, 30000000);  // 0.03s
  double replan_t = (replan_stamp - replan_stamp_).seconds();
  if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
    // Should replan from the hover state
    iniState.col(0) = odom_p;
    iniState.col(1) = odom_v;
  } else {
    // Should replan from the last trajectory
    iniState.col(0) = traj_poly_.getPos(replan_t);
    iniState.col(1) = traj_poly_.getVel(replan_t);
    iniState.col(2) = traj_poly_.getAcc(replan_t);
  }
  replan_state_msg_.header.stamp = node_->now();
  replan_state_msg_.ini_state.resize(9);
  Eigen::Map<Eigen::MatrixXd>(replan_state_msg_.ini_state.data(), 3, 3) = iniState;
  
  // Path searching
  Eigen::Vector3d p_start = iniState.col(0);
  std::vector<Eigen::Vector3d> path, way_pts;
  
  if (generate_new_traj_success) {
    if (land_triger_received_) {
      generate_new_traj_success = env_ptr_->short_astar(p_start, target_p, path);
    } else {
      generate_new_traj_success = env_ptr_->findVisiblePath(p_start, target_predict, way_pts, path);
    }
  }
  
  std::vector<Eigen::Vector3d> visible_ps;
  std::vector<double> thetas;
  Trajectory traj;
  if (generate_new_traj_success) {
    vis_ptr_->visualize_path(path, "astar");
    if (land_triger_received_) {
      for (const auto& p : target_predict) {
        path.push_back(p);
      }
    } else {
      // Generate visible regions
      target_predict.pop_back();
      way_pts.pop_back();
      env_ptr_->generate_visible_regions(target_predict, way_pts, visible_ps, thetas);
      vis_ptr_->visualize_pointcloud(visible_ps, "visible_ps");
      vis_ptr_->visualize_fan_shape_meshes(target_predict, visible_ps, thetas, "visible_region");
      
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predict[i], way_pts[i]);
      }
      vis_ptr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      env_ptr_->pts2path(way_pts, path);
    }
    // Corridor generating
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
    env_ptr_->generateSFC(path, 2.0, hPolys, keyPts);
    env_ptr_->visCorridor(hPolys);
    vis_ptr_->visualize_pairline(keyPts, "keyPts");
    
    // Trajectory optimization
    Eigen::MatrixXd finState;
    finState.setZero(3, 3);
    finState.col(0) = path.back();
    finState.col(1) = target_v;
    if (land_triger_received_) {
      finState.col(0) = target_predict.back();
      generate_new_traj_success = traj_opt_ptr_->generate_traj(iniState, finState, target_predict, hPolys, traj);
    } else {
      generate_new_traj_success = traj_opt_ptr_->generate_traj(iniState, finState,
                                                               target_predict, visible_ps, thetas,
                                                               hPolys, traj);
    }
    vis_ptr_->visualize_traj(traj, "traj");
  }
  
  // Collision check
  bool valid = false;
  if (generate_new_traj_success) {
    valid = validcheck(traj, replan_stamp);
  } else {
    replan_state_msg_.state = -2;
    replan_state_pub_->publish(replan_state_msg_);
  }
  if (valid) {
    force_hover_ = false;
    RCLCPP_WARN(node_->get_logger(), "[planner] REPLAN SUCCESS");
    replan_state_msg_.state = 0;
    replan_state_pub_->publish(replan_state_msg_);
    Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
    double yaw = std::atan2(dp.y(), dp.x());
    if (land_triger_received_) {
      yaw = 2 * std::atan2(target_q.z(), target_q.w());
    }
    pubTraj(traj, yaw, replan_stamp);
    traj_poly_ = traj;
    replan_stamp_ = replan_stamp;
  } else if (force_hover_) {
    RCLCPP_ERROR(node_->get_logger(), "[planner] REPLAN FAILED, HOVERING...");
    replan_state_msg_.state = 1;
    replan_state_pub_->publish(replan_state_msg_);
    return;
  } else if (validcheck(traj_poly_, replan_stamp_)) {
    force_hover_ = true;
    RCLCPP_FATAL(node_->get_logger(), "[planner] EMERGENCY STOP!!!");
    replan_state_msg_.state = 2;
    replan_state_pub_->publish(replan_state_msg_);
    pubHoverP(iniState.col(0), replan_stamp);
    return;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
    replan_state_msg_.state = 3;
    replan_state_pub_->publish(replan_state_msg_);
    return;
  }
  vis_ptr_->visualize_traj(traj, "traj");
}

void TrackingFSM::fakeTimerCallback() {
  heartbeat_pub_->publish(std_msgs::msg::Empty());
  if (!odom_received_ || !map_received_) {
    return;
  }
  
  // Obtain state of odom
  while (odom_lock_.test_and_set())
    ;
  auto odom_msg = odom_msg_;
  odom_lock_.clear();
  Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                         odom_msg.pose.pose.position.y,
                         odom_msg.pose.pose.position.z);
  Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                         odom_msg.twist.twist.linear.y,
                         odom_msg.twist.twist.linear.z);
  
  if (!triger_received_) {
    return;
  }
  
  // Force-hover: waiting for the speed of drone small enough
  if (force_hover_ && odom_v.norm() > 0.1) {
    return;
  }
  
  // Local goal
  Eigen::Vector3d local_goal;
  Eigen::Vector3d delta = goal_ - odom_p;
  if (delta.norm() < 15) {
    local_goal = goal_;
  } else {
    local_goal = delta.normalized() * 15 + odom_p;
  }
  
  // Obtain map
  while (gridmap_lock_.test_and_set())
    ;
  gridmap_ptr_->from_msg(map_msg_);
  replan_state_msg_.occmap = map_msg_;
  gridmap_lock_.clear();
  
  // Determine whether to replan
  bool no_need_replan = false;
  if (!force_hover_ && !wait_hover_) {
    double last_traj_t_rest = traj_poly_.getTotalDuration() - (node_->now() - replan_stamp_).seconds();
    bool new_goal = (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration())).norm() > tracking_dist_;
    if (!new_goal) {
      if (last_traj_t_rest < 1.0) {
        RCLCPP_WARN(node_->get_logger(), "[planner] NEAR GOAL...");
        no_need_replan = true;
      } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
        RCLCPP_WARN(node_->get_logger(), "[planner] NO NEED REPLAN...");
        double t_delta = traj_poly_.getTotalDuration() < 1.0 ? traj_poly_.getTotalDuration() : 1.0;
        double t_yaw = (node_->now() - replan_stamp_).seconds() + t_delta;
        Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
        Eigen::Vector3d dp = un_known_p - odom_p;
        double yaw = std::atan2(dp.y(), dp.x());
        pubTraj(traj_poly_, yaw, replan_stamp_);
        no_need_replan = true;
      }
    }
  }
  
  // Determine whether to pub hover
  if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {
    if (!wait_hover_) {
      pubHoverP(odom_p, node_->now());
      wait_hover_ = true;
    }
    RCLCPP_WARN(node_->get_logger(), "[planner] HOVERING...");
    replan_state_msg_.state = -1;
    replan_state_pub_->publish(replan_state_msg_);
    return;
  } else {
    wait_hover_ = false;
  }
  if (no_need_replan) {
    return;
  }
  
  // Replan state
  Eigen::MatrixXd iniState;
  iniState.setZero(3, 3);
  rclcpp::Time replan_stamp = node_->now() + rclcpp::Duration(0, 30000000);  // 0.03s
  double replan_t = (replan_stamp - replan_stamp_).seconds();
  if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
    // Should replan from the hover state
    iniState.col(0) = odom_p;
    iniState.col(1) = odom_v;
  } else {
    // Should replan from the last trajectory
    iniState.col(0) = traj_poly_.getPos(replan_t);
    iniState.col(1) = traj_poly_.getVel(replan_t);
    iniState.col(2) = traj_poly_.getAcc(replan_t);
  }
  replan_state_msg_.header.stamp = node_->now();
  replan_state_msg_.ini_state.resize(9);
  Eigen::Map<Eigen::MatrixXd>(replan_state_msg_.ini_state.data(), 3, 3) = iniState;
  
  // Generate an extra corridor
  Eigen::Vector3d p_start = iniState.col(0);
  bool need_extra_corridor = iniState.col(1).norm() > 1.0;
  Eigen::MatrixXd hPoly;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
  if (need_extra_corridor) {
    Eigen::Vector3d v_norm = iniState.col(1).normalized();
    line.first = p_start;
    double step = 0.1;
    for (double dx = step; dx < 1.0; dx += step) {
      p_start += step * v_norm;
      if (gridmap_ptr_->isOccupied(p_start)) {
        p_start -= step * v_norm;
        break;
      }
    }
    line.second = p_start;
    env_ptr_->generateOneCorridor(line, 2.0, hPoly);
  }
  
  // Path searching
  std::vector<Eigen::Vector3d> path;
  bool generate_new_traj_success = env_ptr_->astar_search(p_start, local_goal, path);
  Trajectory traj;
  if (generate_new_traj_success) {
    vis_ptr_->visualize_path(path, "astar");
    // Corridor generating
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
    env_ptr_->generateSFC(path, 2.0, hPolys, keyPts);
    if (need_extra_corridor) {
      hPolys.insert(hPolys.begin(), hPoly);
      keyPts.insert(keyPts.begin(), line);
    }
    env_ptr_->visCorridor(hPolys);
    vis_ptr_->visualize_pairline(keyPts, "keyPts");
    
    // Trajectory optimization
    Eigen::MatrixXd finState;
    finState.setZero(3, 3);
    finState.col(0) = path.back();
    generate_new_traj_success = traj_opt_ptr_->generate_traj(iniState, finState, hPolys, traj);
    vis_ptr_->visualize_traj(traj, "traj");
  }
  
  // Collision check
  bool valid = false;
  if (generate_new_traj_success) {
    valid = validcheck(traj, replan_stamp);
  } else {
    replan_state_msg_.state = -2;
    replan_state_pub_->publish(replan_state_msg_);
  }
  if (valid) {
    force_hover_ = false;
    RCLCPP_WARN(node_->get_logger(), "[planner] REPLAN SUCCESS");
    replan_state_msg_.state = 0;
    replan_state_pub_->publish(replan_state_msg_);
    Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
    Eigen::Vector3d dp = un_known_p - odom_p;
    double yaw = std::atan2(dp.y(), dp.x());
    pubTraj(traj, yaw, replan_stamp);
    traj_poly_ = traj;
    replan_stamp_ = replan_stamp;
  } else if (force_hover_) {
    RCLCPP_ERROR(node_->get_logger(), "[planner] REPLAN FAILED, HOVERING...");
    replan_state_msg_.state = 1;
    replan_state_pub_->publish(replan_state_msg_);
    return;
  } else if (!validcheck(traj_poly_, replan_stamp_)) {
    force_hover_ = true;
    RCLCPP_FATAL(node_->get_logger(), "[planner] EMERGENCY STOP!!!");
    replan_state_msg_.state = 2;
    replan_state_pub_->publish(replan_state_msg_);
    pubHoverP(iniState.col(0), replan_stamp);
    return;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
    replan_state_msg_.state = 3;
    replan_state_pub_->publish(replan_state_msg_);
    return;
  }
  vis_ptr_->visualize_traj(traj, "traj");
}

}  // namespace tracker
