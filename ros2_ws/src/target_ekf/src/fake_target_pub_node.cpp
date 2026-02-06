#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>

namespace target_ekf_ros2 {

class FakeTargetPubNode : public rclcpp::Node {
 public:
  FakeTargetPubNode() : rclcpp::Node("fake_target_pub") {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "");
    topic_ = declare_parameter<std::string>("topic", "target");

    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);
    mode_ = declare_parameter<std::string>("mode", "circle");  // circle|static

    center_x_ = declare_parameter<double>("center_x", 0.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    center_z_ = declare_parameter<double>("center_z", 1.0);

    radius_ = declare_parameter<double>("radius", 5.0);
    omega_ = declare_parameter<double>("omega", 0.2);  // rad/s

    static_x_ = declare_parameter<double>("static_x", 0.0);
    static_y_ = declare_parameter<double>("static_y", 0.0);
    static_z_ = declare_parameter<double>("static_z", 1.0);

    // Use RELIABLE to match message_filters default subscriptions (TargetEkfSimNode) and avoid
    // RELIABILITY_QOS_POLICY incompatibilities.
    pub_ = create_publisher<nav_msgs::msg::Odometry>(topic_, rclcpp::QoS(10).reliable());
    t0_ = now();

    const double hz = (rate_hz_ <= 0.0) ? 30.0 : rate_hz_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&FakeTargetPubNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
                "fake_target_pub started (topic=%s mode=%s frame_id=%s rate=%.1fHz)",
                topic_.c_str(), mode_.c_str(), frame_id_.c_str(), hz);
  }

 private:
  static void yawToQuat(double yaw, double& qw, double& qx, double& qy, double& qz) {
    // Z-axis yaw only.
    const double half = 0.5 * yaw;
    qw = std::cos(half);
    qx = 0.0;
    qy = 0.0;
    qz = std::sin(half);
  }

  void onTimer() {
    const double t = (now() - t0_).seconds();

    double x = static_x_;
    double y = static_y_;
    double z = static_z_;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double yaw = 0.0;

    if (mode_ == "circle") {
      const double th = omega_ * t;
      x = center_x_ + radius_ * std::cos(th);
      y = center_y_ + radius_ * std::sin(th);
      z = center_z_;
      vx = -radius_ * omega_ * std::sin(th);
      vy = radius_ * omega_ * std::cos(th);
      vz = 0.0;
      yaw = std::atan2(vy, vx);
    } else if (mode_ == "static") {
      // keep defaults
      yaw = 0.0;
    } else {
      // Unknown mode: publish static but warn occasionally.
      if ((warn_count_++ % 100) == 0) {
        RCLCPP_WARN(get_logger(), "unknown mode '%s', falling back to static", mode_.c_str());
      }
      yaw = 0.0;
    }

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = child_frame_id_;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;

    double qw, qx, qy, qz;
    yawToQuat(yaw, qw, qx, qy, qz);
    msg.pose.pose.orientation.w = qw;
    msg.pose.pose.orientation.x = qx;
    msg.pose.pose.orientation.y = qy;
    msg.pose.pose.orientation.z = qz;

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.linear.z = vz;

    pub_->publish(msg);
  }

  // params
  std::string frame_id_;
  std::string child_frame_id_;
  std::string topic_;
  std::string mode_;
  double rate_hz_{30.0};
  double center_x_{0.0}, center_y_{0.0}, center_z_{1.0};
  double radius_{5.0};
  double omega_{0.2};
  double static_x_{0.0}, static_y_{0.0}, static_z_{1.0};

  // state
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  size_t warn_count_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace target_ekf_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<target_ekf_ros2::FakeTargetPubNode>());
  rclcpp::shutdown();
  return 0;
}

