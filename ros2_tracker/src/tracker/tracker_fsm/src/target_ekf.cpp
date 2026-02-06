#include "tracker_fsm/target_ekf.h"
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace tracker {

void TargetEkf::init(rclcpp::Node::SharedPtr node) {
  node_ = node;
  last_update_stamp_ = node_->now() - rclcpp::Duration(10, 0);

  // Declare and get parameters
  std::vector<double> tmp;
  // cam2body_R already declared by mapper, just get it
  node->get_parameter("cam2body_R", tmp);
  if (tmp.size() == 9) {
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
  }
  node->get_parameter("cam2body_p", tmp);
  if (tmp.size() == 3) {
    cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
  }
  
  // Camera params may already be declared by mapper; use get_parameter_or_set_default pattern
  auto declare_if_not_double = [&](const std::string& name, double default_val) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, default_val);
    }
  };
  auto declare_if_not_int = [&](const std::string& name, int default_val) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<int>(name, default_val);
    }
  };
  auto declare_if_not_bool = [&](const std::string& name, bool default_val) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<bool>(name, default_val);
    }
  };
  declare_if_not_double("cam_fx", 0.0);
  declare_if_not_double("cam_fy", 0.0);
  declare_if_not_double("cam_cx", 0.0);
  declare_if_not_double("cam_cy", 0.0);
  // cam_width/cam_height are int in mapper, keep consistent
  declare_if_not_int("cam_width", 640);
  declare_if_not_int("cam_height", 480);
  declare_if_not_bool("check_fov", false);
  
  node->get_parameter("cam_fx", fx_);
  node->get_parameter("cam_fy", fy_);
  node->get_parameter("cam_cx", cx_);
  node->get_parameter("cam_cy", cy_);
  // Get int params and cast to double
  int cam_w = 640, cam_h = 480;
  node->get_parameter("cam_width", cam_w);
  node->get_parameter("cam_height", cam_h);
  width_ = static_cast<double>(cam_w);
  height_ = static_cast<double>(cam_h);
  node->get_parameter("check_fov", check_fov_);

  // Publishers
  target_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("target_odom", 1);

  // Subscribers
  target_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      node_, "yolo", rclcpp::QoS(1).get_rmw_qos_profile());
  odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      node_, "odom", rclcpp::QoS(100).get_rmw_qos_profile());

  // Synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<OdomSyncPolicy>>(
      OdomSyncPolicy(200), *target_sub_, *odom_sub_);
  sync_->registerCallback(
      std::bind(&TargetEkf::updateCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Timer for prediction
  int ekf_rate = 20;
  node->declare_parameter<int>("ekf_rate", 20);
  node->get_parameter("ekf_rate", ekf_rate);
  ekf_ptr_ = std::make_shared<Ekf>(1.0 / ekf_rate);
  
  predict_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1000 / ekf_rate),
      std::bind(&TargetEkf::predictCallback, this));
}

void TargetEkf::predictCallback() {
  double update_dt = (node_->now() - last_update_stamp_).seconds();
  if (update_dt < 2.0) {
    ekf_ptr_->predict();
    has_target_ = true;
  } else {
    RCLCPP_WARN(node_->get_logger(), "[ekf] too long time no update!");
    has_target_ = false;
    return;
  }
  
  // Publish target odom
  nav_msgs::msg::Odometry target_odom;
  target_odom.header.stamp = node_->now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekf_ptr_->pos().x();
  target_odom.pose.pose.position.y = ekf_ptr_->pos().y();
  target_odom.pose.pose.position.z = ekf_ptr_->pos().z();
  target_odom.twist.twist.linear.x = ekf_ptr_->vel().x();
  target_odom.twist.twist.linear.y = ekf_ptr_->vel().y();
  target_odom.twist.twist.linear.z = ekf_ptr_->vel().z();
  Eigen::Vector3d rpy = ekf_ptr_->rpy();
  Eigen::Quaterniond q = euler2quaternion(rpy);
  target_odom.pose.pose.orientation.w = q.w();
  target_odom.pose.pose.orientation.x = q.x();
  target_odom.pose.pose.orientation.y = q.y();
  target_odom.pose.pose.orientation.z = q.z();
  target_odom_pub_->publish(target_odom);
}

void TargetEkf::updateCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& target_msg,
                                const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  Eigen::Vector3d odom_p, p;
  Eigen::Quaterniond odom_q, q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

  p.x() = target_msg->pose.pose.position.x;
  p.y() = target_msg->pose.pose.position.y;
  p.z() = target_msg->pose.pose.position.z;
  q.w() = target_msg->pose.pose.orientation.w;
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;

  Eigen::Vector3d rpy = quaternion2euler(q);

  // NOTE check whether it's in FOV
  if (check_fov_) {
    Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) {
      return;
    }
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
    if (x < 0 || x > height_) {
      return;
    }
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
    if (y < 0 || y > width_) {
      return;
    }
  }

  // Update target odom
  double update_dt = (node_->now() - last_update_stamp_).seconds();
  if (update_dt > 5.0) {
    ekf_ptr_->reset(p, rpy);
    RCLCPP_WARN(node_->get_logger(), "[ekf] reset!");
    has_target_ = true;
  } else if (ekf_ptr_->update(p, rpy)) {
    has_target_ = true;
    // RCLCPP_WARN(node_->get_logger(), "[ekf] update!");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[ekf] update invalid!");
    has_target_ = false;
    return;
  }
  last_update_stamp_ = node_->now();
}

Eigen::Quaterniond TargetEkf::getTargetQuat() const {
  if (!ekf_ptr_) {
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Vector3d rpy = ekf_ptr_->rpy();
  return euler2quaternion(rpy);
}

}  // namespace tracker
