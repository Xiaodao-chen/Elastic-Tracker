#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <object_detection_msgs/msg/bounding_boxes.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace target_ekf_ros2 {

using object_detection_msgs::msg::BoundingBoxes;
using nav_msgs::msg::Odometry;

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  explicit Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 3);
    C.setZero(3, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    K = C;
    Qt.setIdentity(3, 3);
    Rt.setIdentity(3, 3);
    Qt(0, 0) = 4;
    Qt(1, 1) = 4;
    Qt(2, 2) = 1;
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    x.setZero(6);
  }
  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
  }
  inline void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }
  inline bool checkValid(const Eigen::Vector3d& z) const {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    return x_tmp.tail(3).norm() <= vmax;
  }
  inline void update(const Eigen::Vector3d& z) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }
  inline Eigen::Vector3d pos() const { return x.head(3); }
  inline Eigen::Vector3d vel() const { return x.tail(3); }
};

class TargetEkfNode : public rclcpp::Node {
 public:
  TargetEkfNode() : rclcpp::Node("target_ekf") {
    declare_parameter<std::vector<double>>("cam2body_R", std::vector<double>{});
    declare_parameter<std::vector<double>>("cam2body_p", std::vector<double>{});
    declare_parameter<double>("cam_fx", 0.0);
    declare_parameter<double>("cam_fy", 0.0);
    declare_parameter<double>("cam_cx", 0.0);
    declare_parameter<double>("cam_cy", 0.0);
    declare_parameter<double>("pitch_thr", 30.0);
    declare_parameter<int>("ekf_rate", 20);

    loadParams();

    target_odom_pub_ = create_publisher<Odometry>("target_odom", 10);
    yolo_odom_pub_ = create_publisher<Odometry>("yolo_odom", 10);

    last_update_stamp_ = now() - rclcpp::Duration(10, 0);
  }

  void start() {
    const int ekf_rate = ekf_rate_;
    ekf_ = std::make_shared<Ekf>(1.0 / static_cast<double>(ekf_rate));

    yolo_sub_.subscribe(shared_from_this(), "yolo");
    odom_sub_.subscribe(shared_from_this(), "odom");

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(200), yolo_sub_, odom_sub_);
    sync_->registerCallback(std::bind(&TargetEkfNode::update_state_callback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    using namespace std::chrono_literals;
    predict_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / static_cast<double>(ekf_rate)),
      std::bind(&TargetEkfNode::predict_state_callback, this));

    RCLCPP_INFO(get_logger(), "target_ekf started (ekf_rate=%d)", ekf_rate);
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<BoundingBoxes, Odometry>;
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
    get_parameter("pitch_thr", pitch_thr_);
    get_parameter("ekf_rate", ekf_rate_);
  }

  void predict_state_callback() {
    const double update_dt = (now() - last_update_stamp_).seconds();
    if (update_dt < 2.0) {
      ekf_->predict();
    } else {
      RCLCPP_WARN(get_logger(), "too long time no update!");
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
    target_odom.pose.pose.orientation.w = 1.0;
    target_odom_pub_->publish(target_odom);
  }

  void update_state_callback(const BoundingBoxes::ConstSharedPtr& bboxes_msg,
                             const Odometry::ConstSharedPtr& odom_msg) {
    if (!bboxes_msg || bboxes_msg->bounding_boxes.empty()) {
      return;
    }

    Eigen::Vector3d odom_p;
    Eigen::Quaterniond odom_q;
    odom_p(0) = odom_msg->pose.pose.position.x;
    odom_p(1) = odom_msg->pose.pose.position.y;
    odom_p(2) = odom_msg->pose.pose.position.z;
    odom_q.w() = odom_msg->pose.pose.orientation.w;
    odom_q.x() = odom_msg->pose.pose.orientation.x;
    odom_q.y() = odom_msg->pose.pose.orientation.y;
    odom_q.z() = odom_msg->pose.pose.orientation.z;

    Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
    Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

    const auto& yolo_bbox = bboxes_msg->bounding_boxes.front();
    const double xmin = yolo_bbox.xmin;
    const double xmax = yolo_bbox.xmax;
    const double ymin = yolo_bbox.ymin;
    const double ymax = yolo_bbox.ymax;

    const double pixel_thr = 30;
    if (ymin < pixel_thr || ymax > 480 - pixel_thr) {
      RCLCPP_ERROR(get_logger(), "pitch out of range!");
      return;
    }

    const double height = ymax - ymin;
    const double depth = 0.7 / height * fy_;
    const double y = ((ymin + ymax) * 0.5 - cy_) * depth / fy_;
    const double x = ((xmin + xmax) * 0.5 - cx_) * depth / fx_;
    Eigen::Vector3d p(x, y, depth);
    p = cam_q * p + cam_p;

    // publish yolo odom
    Odometry yolo_odom;
    yolo_odom.header.stamp = bboxes_msg->header.stamp;
    yolo_odom.header.frame_id = "world";
    yolo_odom.pose.pose.orientation.w = 1.0;
    yolo_odom.pose.pose.position.x = p.x();
    yolo_odom.pose.pose.position.y = p.y();
    yolo_odom.pose.pose.position.z = p.z();
    yolo_odom_pub_->publish(yolo_odom);

    // update target odom
    const double update_dt = (now() - last_update_stamp_).seconds();
    if (update_dt > 3.0) {
      ekf_->reset(p);
      RCLCPP_WARN(get_logger(), "ekf reset!");
    } else if (ekf_->checkValid(p)) {
      ekf_->update(p);
    } else {
      RCLCPP_ERROR(get_logger(), "update invalid!");
      return;
    }
    last_update_stamp_ = now();
  }

  // pubs
  rclcpp::Publisher<Odometry>::SharedPtr target_odom_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr yolo_odom_pub_;

  // params
  Eigen::Matrix3d cam2body_R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d cam2body_p_{Eigen::Vector3d::Zero()};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  double pitch_thr_{30.0};
  int ekf_rate_{20};

  // state
  rclcpp::Time last_update_stamp_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<Ekf> ekf_;

  // sync
  message_filters::Subscriber<BoundingBoxes> yolo_sub_;
  message_filters::Subscriber<Odometry> odom_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::TimerBase::SharedPtr predict_timer_;
};

}  // namespace target_ekf_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<target_ekf_ros2::TargetEkfNode>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
