#include "tracker_fsm/occ_grid_mapper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tracker {

void OccGridMapper::init(rclcpp::Node::SharedPtr node) {
  node_ = node;
  
  // Load cam2body transformation parameters
  std::vector<double> tmp;
  node_->declare_parameter("cam2body_R", std::vector<double>());
  node_->get_parameter("cam2body_R", tmp);
  if (tmp.size() == 9) {
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
  }
  node_->declare_parameter("cam2body_p", std::vector<double>());
  node_->get_parameter("cam2body_p", tmp);
  if (tmp.size() == 3) {
    cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
  }
  
  // Check if using global map mode
  node_->declare_parameter<bool>("use_global_map", false);
  node_->get_parameter("use_global_map", use_global_map_);
  
  if (use_global_map_) {
    // Global map mode parameters
    double x, y, z, res;
    node_->declare_parameter<double>("x_length", 0.0);
    node_->declare_parameter<double>("y_length", 0.0);
    node_->declare_parameter<double>("z_length", 0.0);
    node_->declare_parameter<double>("resolution", 0.0);
    node_->declare_parameter<int>("inflate_size", 0);
    
    node_->get_parameter("x_length", x);
    node_->get_parameter("y_length", y);
    node_->get_parameter("z_length", z);
    node_->get_parameter("resolution", res);
    node_->get_parameter("inflate_size", inflate_size_);
    
    gridmap_.setup(res, Eigen::Vector3d(x, y, z), 10, true);
  } else {
    // Depth camera mode parameters
    node_->declare_parameter<double>("camera_rate", 30.0);
    node_->declare_parameter<double>("camera_range", 10.0);
    node_->declare_parameter<int>("cam_width", 640);
    node_->declare_parameter<int>("cam_height", 480);
    node_->declare_parameter<double>("cam_fx", 320.0);
    node_->declare_parameter<double>("cam_fy", 320.0);
    node_->declare_parameter<double>("cam_cx", 320.0);
    node_->declare_parameter<double>("cam_cy", 240.0);
    node_->declare_parameter<double>("depth_scaling_factor", 1000.0);
    
    node_->get_parameter("camera_rate", cam_config_.rate);
    node_->get_parameter("camera_range", cam_config_.range);
    node_->get_parameter("cam_width", cam_config_.width);
    node_->get_parameter("cam_height", cam_config_.height);
    node_->get_parameter("cam_fx", cam_config_.fx);
    node_->get_parameter("cam_fy", cam_config_.fy);
    node_->get_parameter("cam_cx", cam_config_.cx);
    node_->get_parameter("cam_cy", cam_config_.cy);
    node_->get_parameter("depth_scaling_factor", cam_config_.depth_scaling_factor);
    
    // Mapping parameters
    double res;
    Eigen::Vector3d map_size;
    node_->declare_parameter<int>("down_sample_factor", 1);
    node_->declare_parameter<double>("resolution", 0.1);
    node_->declare_parameter<double>("local_x", 10.0);
    node_->declare_parameter<double>("local_y", 10.0);
    node_->declare_parameter<double>("local_z", 5.0);
    node_->declare_parameter<int>("inflate_size", 1);
    
    node_->get_parameter("down_sample_factor", down_sample_factor_);
    node_->get_parameter("resolution", res);
    node_->get_parameter("local_x", map_size.x());
    node_->get_parameter("local_y", map_size.y());
    node_->get_parameter("local_z", map_size.z());
    node_->get_parameter("inflate_size", inflate_size_);
    
    gridmap_.setup(res, map_size, cam_config_.range);
    
    // Depth filter parameters
    node_->declare_parameter<double>("depth_filter_tolerance", 0.1);
    node_->declare_parameter<double>("depth_filter_mindist", 0.1);
    node_->declare_parameter<int>("depth_filter_margin", 5);
    
    node_->get_parameter("depth_filter_tolerance", depth_filter_tolerance_);
    node_->get_parameter("depth_filter_mindist", depth_filter_mindist_);
    node_->get_parameter("depth_filter_margin", depth_filter_margin_);
    
    // Raycasting parameters
    int p_min, p_max, p_hit, p_mis, p_occ, p_def;
    node_->declare_parameter<int>("p_min", -199);
    node_->declare_parameter<int>("p_max", 220);
    node_->declare_parameter<int>("p_hit", 62);
    node_->declare_parameter<int>("p_mis", 62);
    node_->declare_parameter<int>("p_occ", 139);
    node_->declare_parameter<int>("p_def", -199);
    
    node_->get_parameter("p_min", p_min);
    node_->get_parameter("p_max", p_max);
    node_->get_parameter("p_hit", p_hit);
    node_->get_parameter("p_mis", p_mis);
    node_->get_parameter("p_occ", p_occ);
    node_->get_parameter("p_def", p_def);
    
    gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
  }
  
  gridmap_.inflate_size = inflate_size_;
  
  // Use mask parameter
  node_->declare_parameter<bool>("use_mask", false);
  node_->get_parameter("use_mask", use_mask_);
  
  // Create publisher
  gridmap_pub_ = node_->create_publisher<quadrotor_msgs::msg::OccMap3d>("gridmap_inflate", rclcpp::QoS(10));
  
  if (use_global_map_) {
    RCLCPP_WARN(node_->get_logger(), "[mapping] GLOBAL MAP mode enabled");
    // Global map mode: subscribe to global map pointcloud
    map_pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "global_map", rclcpp::QoS(10),
        std::bind(&OccGridMapper::globalMapCallback, this, std::placeholders::_1));
    
    // Timer to periodically publish the map
    global_map_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&OccGridMapper::globalMapTimerCallback, this));
  } else {
    RCLCPP_WARN(node_->get_logger(), "[mapping] DEPTH mode enabled, subscribing to depth + odom");
    // Depth mode: synchronize depth image and odometry
    // Use BEST_EFFORT reliability for depth to match both:
    //   - pcl_render_node in simulation (RELIABLE by default, but BEST_EFFORT sub can receive)
    //   - RealSense camera driver (publishes BEST_EFFORT by default)
    // NOTE: BEST_EFFORT subscriber is compatible with BOTH RELIABLE and BEST_EFFORT publishers
    rmw_qos_profile_t depth_qos = rmw_qos_profile_sensor_data;
    depth_qos.depth = 10;
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_, "depth", depth_qos);
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        node_, "odom", rclcpp::QoS(50).get_rmw_qos_profile());
    
    sync_ = std::make_shared<message_filters::Synchronizer<ImageOdomSyncPolicy>>(
        ImageOdomSyncPolicy(100), *depth_sub_, *odom_sub_);
    sync_->registerCallback(
        std::bind(&OccGridMapper::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Periodic timer to check if depth messages are being received (debug)
    depth_debug_timer_ = node_->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
          if (!depth_received_) {
            RCLCPP_WARN(node_->get_logger(), "[mapping] WARNING: No depth+odom sync callback received yet! Check depth topic.");
          }
        });
  }
  
  if (use_mask_) {
    target_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "target", rclcpp::QoS(10),
        std::bind(&OccGridMapper::targetOdomCallback, this, std::placeholders::_1));
  }
}

void OccGridMapper::depthOdomCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  if (!depth_received_) {
    RCLCPP_INFO(node_->get_logger(), "[mapping] First depth+odom sync received! encoding=%s %dx%d",
                depth_msg->encoding.c_str(), depth_msg->width, depth_msg->height);
    depth_received_ = true;
  }
  if (callback_lock_.test_and_set()) {
    return;
  }
  
  Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                         odom_msg->pose.pose.position.y,
                         odom_msg->pose.pose.position.z);
  Eigen::Quaterniond body_q(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
  Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);
  
  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, cam_config_.depth_scaling_factor);
  }
  cv::Mat depth_img = depth_ptr->image;
  
  int nr = depth_img.rows;
  int nc = depth_img.cols;
  std::vector<Eigen::Vector3d> obs_pts;
  
  // Process depth image and convert to 3D points
  for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
    for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
      // (x,y,z) in camera frame
      double z = (depth_img.at<uint16_t>(i, j)) / cam_config_.depth_scaling_factor;
      if (depth_img.at<uint16_t>(i, j) == 0) {
        z = cam_config_.range + 0.5;
      }
      if (std::isnan(z) || std::isinf(z))
        continue;
      if (z < depth_filter_mindist_) {
        continue;
      }
      double y = (i - cam_config_.cy) * z / cam_config_.fy;
      double x = (j - cam_config_.cx) * z / cam_config_.fx;
      Eigen::Vector3d p(x, y, z);
      p = cam_q * p + cam_p;
      bool good_point = true;
      if (get_first_frame_) {
        // NOTE depth filter:
        Eigen::Vector3d p_rev_proj =
            last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
        double vv = p_rev_proj.y() * cam_config_.fy / p_rev_proj.z() + cam_config_.cy;
        double uu = p_rev_proj.x() * cam_config_.fx / p_rev_proj.z() + cam_config_.cx;
        if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
          double drift_dis = fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / cam_config_.depth_scaling_factor - p_rev_proj.z());
          if (drift_dis > depth_filter_tolerance_) {
            good_point = false;
          }
        }
      }
      if (good_point) {
        obs_pts.push_back(p);
      }
    }
  }
  last_depth_ = depth_img;
  last_cam_p_ = cam_p;
  last_cam_q_ = cam_q;
  get_first_frame_ = true;
  gridmap_.updateMap(cam_p, obs_pts);
  
  // NOTE use mask
  if (use_mask_) {  // mask target
    while (target_lock_.test_and_set())
      ;
    Eigen::Vector3d ld = target_odom_;
    Eigen::Vector3d ru = target_odom_;
    ld.x() -= 0.5;
    ld.y() -= 0.5;
    ld.z() -= 1.0;
    ru.x() += 0.5;
    ru.y() += 0.5;
    ru.z() += 1.0;
    gridmap_.setFree(ld, ru);
    target_lock_.clear();
  }
  
  // Publish gridmap
  quadrotor_msgs::msg::OccMap3d gridmap_msg;
  gridmap_msg.header.frame_id = "world";
  gridmap_msg.header.stamp = node_->now();
  gridmap_.to_msg(gridmap_msg);
  gridmap_pub_->publish(gridmap_msg);
  
  callback_lock_.clear();
}

void OccGridMapper::targetOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  while (target_lock_.test_and_set())
    ;
  target_odom_.x() = msg->pose.pose.position.x;
  target_odom_.y() = msg->pose.pose.position.y;
  target_odom_.z() = msg->pose.pose.position.z;
  target_lock_.clear();
}

void OccGridMapper::globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (map_received_) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromROSMsg(*msg, point_cloud);
  for (const auto& pt : point_cloud) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    gridmap_.setOcc(p);
  }
  gridmap_.inflate(inflate_size_);
  RCLCPP_WARN(node_->get_logger(), "[mapping] GLOBAL MAP RECEIVED!");
  map_received_ = true;
}

void OccGridMapper::globalMapTimerCallback() {
  if (!map_received_) {
    return;
  }
  quadrotor_msgs::msg::OccMap3d gridmap_msg;
  gridmap_.to_msg(gridmap_msg);
  gridmap_msg.header.frame_id = "world";
  gridmap_msg.header.stamp = node_->now();
  gridmap_pub_->publish(gridmap_msg);
}

}  // namespace tracker
