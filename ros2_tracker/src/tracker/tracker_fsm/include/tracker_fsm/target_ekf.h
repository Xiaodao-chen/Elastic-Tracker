#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Geometry>

// EKF math functions and struct (from target_ekf.hpp)
inline Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d& euler) {
  double cr = cos(euler(0) / 2);
  double sr = sin(euler(0) / 2);
  double cp = cos(euler(1) / 2);
  double sp = sin(euler(1) / 2);
  double cy = cos(euler(2) / 2);
  double sy = sin(euler(2) / 2);
  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
  return q;
}

inline Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d m = q.toRotationMatrix();
  Eigen::Vector3d rpy;
  rpy.x() = atan2(m(2, 1), m(2, 2));
  rpy.y() = asin(-m(2, 0));
  rpy.z() = atan2(m(1, 0), m(0, 0));
  return rpy;
}

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  // states: x, y, z, vx, vy, vz, roll, pitch, yaw

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(9, 9);
    Sigma.setZero(9, 9);
    B.setZero(9, 6);
    C.setZero(6, 9);
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
    B(6, 3) = dt;
    B(7, 4) = dt;
    B(8, 5) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 6) = 1;
    C(4, 7) = 1;
    C(5, 8) = 1;
    K = C;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt(0, 0) = 4;    // x
    Qt(1, 1) = 4;    // y
    Qt(2, 2) = 1;    // z
    Qt(3, 3) = 1;    // roll
    Qt(4, 4) = 1;    // pitch
    Qt(5, 5) = 0.1;  // yaw
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    Rt(3, 3) = 0.01;
    Rt(4, 4) = 0.01;
    Rt(5, 5) = 0.01;
    x.setZero(9);
  }
  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
    return;
  }
  inline void reset(const Eigen::Vector3d& z, const Eigen::Vector3d& z_rpy) {
    x.setZero();
    x.head(3) = z;
    x.tail(3) = z_rpy;
    Sigma.setZero();
  }
  inline bool update(const Eigen::Vector3d& z, const Eigen::Vector3d& z_rqp) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd zz(6);
    zz.head(3) = z;
    zz.tail(3) = z_rqp;
    Eigen::VectorXd x_tmp = x + K * (zz - C * x);
    // NOTE check valid
    static double vmax = 4;
    if (x_tmp.middleRows(3, 3).norm() > vmax) {
      return false;
    }
    Eigen::Vector3d d_rpy = x.tail(3) - z_rqp;
    x.tail(3).x() = d_rpy.x() > M_PI ? x.tail(3).x() - 2 * M_PI : x.tail(3).x();
    x.tail(3).y() = d_rpy.y() > M_PI ? x.tail(3).y() - 2 * M_PI : x.tail(3).y();
    x.tail(3).z() = d_rpy.z() > M_PI ? x.tail(3).z() - 2 * M_PI : x.tail(3).z();
    x.tail(3).x() = d_rpy.x() < -M_PI ? x.tail(3).x() + 2 * M_PI : x.tail(3).x();
    x.tail(3).y() = d_rpy.y() < -M_PI ? x.tail(3).y() + 2 * M_PI : x.tail(3).y();
    x.tail(3).z() = d_rpy.z() < -M_PI ? x.tail(3).z() + 2 * M_PI : x.tail(3).z();
    x = x + K * (zz - C * x);
    Sigma = Sigma - K * C * Sigma;
    return true;
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.middleRows(3, 3);
  }
  inline const Eigen::Vector3d rpy() const {
    return x.tail(3);
  }
};

namespace tracker {

class TargetEkf {
public:
  TargetEkf() {}
  void init(rclcpp::Node::SharedPtr node);
  
  bool hasTarget() const { return has_target_; }
  Eigen::Vector3d getTargetPos() const { return ekf_ptr_ ? ekf_ptr_->pos() : Eigen::Vector3d::Zero(); }
  Eigen::Vector3d getTargetVel() const { return ekf_ptr_ ? ekf_ptr_->vel() : Eigen::Vector3d::Zero(); }
  Eigen::Vector3d getTargetRpy() const { return ekf_ptr_ ? ekf_ptr_->rpy() : Eigen::Vector3d::Zero(); }
  Eigen::Quaterniond getTargetQuat() const;
  
private:
  rclcpp::Node::SharedPtr node_;
  
  // EKF
  std::shared_ptr<Ekf> ekf_ptr_;
  rclcpp::Time last_update_stamp_;
  bool has_target_ = false;
  
  // Camera params for FOV check
  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  double fx_, fy_, cx_, cy_, width_, height_;
  bool check_fov_ = false;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_;
  
  // Subscribers (sim mode: sync two odoms)
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> target_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry> OdomSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<OdomSyncPolicy>> sync_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr predict_timer_;
  
  void predictCallback();
  void updateCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& target_msg,
                      const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);
};

}  // namespace tracker
