#pragma once
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>

#include "tracker_fsm/mapping.h"

#include <atomic>

namespace tracker {

struct CamConfig {
  double rate;
  double range;
  int width;
  int height;
  double fx, fy, cx, cy;
  double depth_scaling_factor;
};

class OccGridMapper {
public:
  OccGridMapper() {}
  void init(rclcpp::Node::SharedPtr node);
  mapping::OccGridMap& getMap() { return gridmap_; }
  
private:
  rclcpp::Node::SharedPtr node_;
  CamConfig cam_config_;
  int down_sample_factor_;
  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  
  // depth filter
  Eigen::Vector3d last_cam_p_;
  Eigen::Quaterniond last_cam_q_;
  bool get_first_frame_ = false;
  cv::Mat last_depth_;
  double depth_filter_tolerance_;
  double depth_filter_mindist_;
  int depth_filter_margin_;
  
  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;
  
  // For depth mode
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> ImageOdomSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ImageOdomSyncPolicy>> sync_;
  
  // For global map mode
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_sub_;
  bool map_received_ = false;
  bool use_global_map_ = false;
  
  // Debug
  rclcpp::TimerBase::SharedPtr depth_debug_timer_;
  bool depth_received_ = false;
  
  // For target mask
  bool use_mask_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d target_odom_;
  
  rclcpp::Publisher<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_pub_;
  
  mapping::OccGridMap gridmap_;
  int inflate_size_;
  
  void depthOdomCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                          const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);
  void targetOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void globalMapTimerCallback();
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace tracker
