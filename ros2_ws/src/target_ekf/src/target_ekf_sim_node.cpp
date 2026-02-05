#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <target_ekf/target_ekf.hpp>

namespace target_ekf_ros2 {

using nav_msgs::msg::Odometry;

class TargetEkfSimNode : public rclcpp::Node {
 public:
  TargetEkfSimNode() : rclcpp::Node("target_ekf_sim") {
    declare_parameter<std::vector<double>>("cam2body_R", std::vector<double>{});
    declare_parameter<std::vector<double>>("cam2body_p", std::vector<double>{});
    declare_parameter<double>("cam_fx", 0.0);
    declare_parameter<double>("cam_fy", 0.0);
    declare_parameter<double>("cam_cx", 0.0);
    declare_parameter<double>("cam_cy", 0.0);
    declare_parameter<double>("cam_width", 640.0);
    declare_parameter<double>("cam_height", 480.0);
    declare_parameter<double>("pitch_thr", 30.0);
    declare_parameter<bool>("check_fov", false);
    declare_parameter<int>("ekf_rate", 20);

    loadParams();
    target_odom_pub_ = create_publisher<Odometry>("target_odom", 10);
    yolo_odom_pub_ = create_publisher<Odometry>("yolo_odom", 10);
    last_update_stamp_ = now() - rclcpp::Duration(10, 0);
  }

  void start() {
    ekf_ = std::make_shared<Ekf>(1.0 / static_cast<double>(ekf_rate_));

    yolo_sub_.subscribe(shared_from_this(), "yolo");
    odom_sub_.subscribe(shared_from_this(), "odom");
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(200), yolo_sub_, odom_sub_);
    sync_->registerCallback(std::bind(&TargetEkfSimNode::update_state_callback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    predict_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / static_cast<double>(ekf_rate_)),
      std::bind(&TargetEkfSimNode::predict_state_callback, this));
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Odometry, Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void loadParams() {
    std::vector<double> tmp;
    get_parameter("cam2body_R", tmp);
    if (tmp.size() == 9) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(tmp.data());
    } else {
      cam2body_R_.setIdentity();
    }
    get_parameter("cam2body_p", tmp);
    if (tmp.size() == 3) {
      cam2body_p_ = Eigen::Map<const Eigen::Vector3d>(tmp.data());
    } else {
      cam2body_p_.setZero();
    }
    get_parameter("cam_fx", fx_);
    get_parameter("cam_fy", fy_);
    get_parameter("cam_cx", cx_);
    get_parameter("cam_cy", cy_);
    get_parameter("cam_width", width_);
    get_parameter("cam_height", height_);
    get_parameter("pitch_thr", pitch_thr_);
    get_parameter("check_fov", check_fov_);
    get_parameter("ekf_rate", ekf_rate_);
  }

  void predict_state_callback() {
    const double update_dt = (now() - last_update_stamp_).seconds();
    if (update_dt < 2.0) {
      ekf_->predict();
    } else {
      RCLCPP_WARN(get_logger(), "[ekf] too long time no update!");
      return;
    }

    Odometry target_odom;
    target_odom.header.stamp = now();
    target_odom.header.frame_id = "world";
    target_odom.pose.pose.position.x = ekf_->pos().x();
    target_odom.pose.pose.position.y = ekf_->pos().y();
    target_odom.pose.pose.position.z = ekf_->pos().z();
    target_odom.twist.twist.linear.x = ekf_->vel().x();
    target_odom.twist.twist.linear.y = ekf_->vel().y();
    target_odom.twist.twist.linear.z = ekf_->vel().z();
    Eigen::Vector3d rpy = ekf_->rpy();
    Eigen::Quaterniond q = euler2quaternion(rpy);
    target_odom.pose.pose.orientation.w = q.w();
    target_odom.pose.pose.orientation.x = q.x();
    target_odom.pose.pose.orientation.y = q.y();
    target_odom.pose.pose.orientation.z = q.z();
    target_odom_pub_->publish(target_odom);
  }

  void update_state_callback(const Odometry::ConstSharedPtr& target_msg,
                             const Odometry::ConstSharedPtr& odom_msg) {
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

    if (check_fov_) {
      Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
      if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) {
        return;
      }
      const double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
      if (x < 0 || x > height_) {
        return;
      }
      const double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
      if (y < 0 || y > width_) {
        return;
      }
    }

    const double update_dt = (now() - last_update_stamp_).seconds();
    if (update_dt > 5.0) {
      ekf_->reset(p, rpy);
      RCLCPP_WARN(get_logger(), "[ekf] reset!");
    } else if (ekf_->update(p, rpy)) {
      // ok
    } else {
      RCLCPP_ERROR(get_logger(), "[ekf] update invalid!");
      return;
    }
    last_update_stamp_ = now();

    // publish raw yolo for debug
    yolo_odom_pub_->publish(*target_msg);
  }

  // pubs
  rclcpp::Publisher<Odometry>::SharedPtr target_odom_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr yolo_odom_pub_;

  // params
  Eigen::Matrix3d cam2body_R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d cam2body_p_{Eigen::Vector3d::Zero()};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0}, width_{640.0}, height_{480.0};
  double pitch_thr_{30.0};
  bool check_fov_{false};
  int ekf_rate_{20};

  rclcpp::Time last_update_stamp_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<Ekf> ekf_;

  message_filters::Subscriber<Odometry> yolo_sub_;
  message_filters::Subscriber<Odometry> odom_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::TimerBase::SharedPtr predict_timer_;
};

}  // namespace target_ekf_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<target_ekf_ros2::TargetEkfSimNode>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
