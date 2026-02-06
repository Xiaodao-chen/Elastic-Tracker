#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/replan_state.hpp>
#include <quadrotor_msgs/msg/occ_map3d.hpp>

#include <mutex>

#include "tracker_fsm/mapping.h"
#include "tracker_fsm/occ_grid_mapper.h"
#include "tracker_fsm/target_ekf.h"
#include "tracker_fsm/tracking_visualization.h"
#include "tracker_fsm/env.hpp"
#include "tracker_fsm/prediction.hpp"
#include <traj_opt/traj_opt.h>
#include <traj_opt/poly_traj_utils.hpp>

namespace tracker {

class TrackingFSM {
public:
  TrackingFSM() {}
  void init(rclcpp::Node::SharedPtr node);
  
private:
  rclcpp::Node::SharedPtr node_;
  
  // Sub-modules (composition pattern)
  std::shared_ptr<mapping::OccGridMap> gridmap_ptr_;
  OccGridMapper mapper_;
  TargetEkf target_ekf_;
  std::shared_ptr<env::Env> env_ptr_;
  std::shared_ptr<visualization::Visualization> vis_ptr_;
  std::shared_ptr<traj_opt::TrajOpt> traj_opt_ptr_;
  std::shared_ptr<prediction::Predict> pre_ptr_;
  
  // Parameters
  double tracking_dur_, tracking_dist_, tolerance_d_;
  bool debug_, fake_;
  int plan_hz_;
  int traj_id_ = 0;
  
  // State
  bool odom_received_ = false;
  bool map_received_ = false;
  bool triger_received_ = false;
  bool target_received_ = false;
  bool force_hover_ = true;
  bool wait_hover_ = false;
  bool land_triger_received_ = false;
  
  // Data (with locks)
  mutable std::mutex odom_mutex_;
  mutable std::mutex target_mutex_;
  mutable std::mutex gridmap_mutex_;
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry target_msg_;
  quadrotor_msgs::msg::OccMap3d map_msg_;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;
  
  // Trajectory
  Trajectory traj_poly_;
  rclcpp::Time replan_stamp_;
  
  // ROS2 interfaces
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, target_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr triger_sub_, land_triger_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PolyTraj>::SharedPtr traj_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::ReplanState>::SharedPtr replan_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  
  quadrotor_msgs::msg::ReplanState replan_state_msg_;
  
  // Callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gridmapCallback(const quadrotor_msgs::msg::OccMap3d::SharedPtr msg);
  void trigerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void landTrigerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  // Planning functions
  void planTimerCallback();   // tracking mode
  void fakeTimerCallback();   // navigation mode
  
  // Helpers
  bool validcheck(const Trajectory& traj, const rclcpp::Time& t_start, double check_dur = 1.0);
  void pubTraj(const Trajectory& traj, double yaw, const rclcpp::Time& stamp);
  void pubHoverP(const Eigen::Vector3d& p, const rclcpp::Time& stamp);
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace tracker
