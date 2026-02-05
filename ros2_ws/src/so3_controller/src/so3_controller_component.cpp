#include <so3_controller/so3_controller.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace so3_controller {

class SO3ControllerComponent : public rclcpp::Node {
 public:
  explicit SO3ControllerComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("so3_controller", options) {
    // parameters
    const double mass = declare_parameter<double>("mass", 1.0);
    const double g    = declare_parameter<double>("g", 9.81);
    so3ControlPtr_ = std::make_shared<SO3Controller>(mass, g);

    so3cmd_.header.frame_id = declare_parameter<std::string>("frame_id", "world");

    so3cmd_.k_r[0] = declare_parameter<double>("gains/rot/x", 1.0);
    so3cmd_.k_r[1] = declare_parameter<double>("gains/rot/y", 1.0);
    so3cmd_.k_r[2] = declare_parameter<double>("gains/rot/z", 1.0);
    so3cmd_.k_om[0] = declare_parameter<double>("gains/ang/x", 0.1);
    so3cmd_.k_om[1] = declare_parameter<double>("gains/ang/y", 0.1);
    so3cmd_.k_om[2] = declare_parameter<double>("gains/ang/z", 0.1);

    so3cmd_.aux.kf_correction = declare_parameter<double>("corrections/z", 0.0);
    so3cmd_.aux.angle_corrections[0] = declare_parameter<double>("corrections/r", 0.0);
    so3cmd_.aux.angle_corrections[1] = declare_parameter<double>("corrections/p", 0.0);

    so3cmd_.aux.enable_motors = declare_parameter<bool>("enable_motors", true);
    so3cmd_.aux.use_external_yaw = declare_parameter<bool>("use_external_yaw", false);

    // pubs/subs
    position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "position_cmd", 10,
      std::bind(&SO3ControllerComponent::positionCmdCb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&SO3ControllerComponent::odomCb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&SO3ControllerComponent::imuCb, this, std::placeholders::_1));

    so3cmd_pub_ = create_publisher<quadrotor_msgs::msg::SO3Command>("so3cmd", 10);

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SO3ControllerComponent::timerCb, this));
  }

 private:
  void timerCb() {
    if (position_cmd_received_flag_) {
      position_cmd_received_flag_ = false;
      return;
    }

    Eigen::Vector3d des_vel(0, 0, 0);
    Eigen::Vector3d des_acc(0, 0, 0);
    Eigen::Vector3d kx(5.7, 5.7, 6.2);
    Eigen::Vector3d kv(3.4, 3.4, 4.0);
    so3ControlPtr_->calculateControl(des_pos_, des_vel, des_acc, des_yaw_, 0, kx, kv);

    const Eigen::Vector3d& f = so3ControlPtr_->getF();
    const Eigen::Quaterniond& q = so3ControlPtr_->getQ();

    so3cmd_.header.stamp = now();
    so3cmd_.force.x = f(0);
    so3cmd_.force.y = f(1);
    so3cmd_.force.z = f(2);
    so3cmd_.orientation.x = q.x();
    so3cmd_.orientation.y = q.y();
    so3cmd_.orientation.z = q.z();
    so3cmd_.orientation.w = q.w();
    so3cmd_pub_->publish(so3cmd_);
  }

  void positionCmdCb(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
    position_cmd_received_flag_ = true;

    Eigen::Vector3d des_pos(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Vector3d des_vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    Eigen::Vector3d des_acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    Eigen::Vector3d kx(msg->kx[0], msg->kx[1], msg->kx[2]);
    Eigen::Vector3d kv(msg->kv[0], msg->kv[1], msg->kv[2]);
    if (msg->kx[0] == 0) {
      kx(0) = 5.7;
      kx(1) = 5.7;
      kx(2) = 6.2;
      kv(0) = 3.4;
      kv(1) = 3.4;
      kv(2) = 4.0;
    }
    const double des_yaw = msg->yaw;
    const double des_yaw_dot = msg->yaw_dot;

    so3ControlPtr_->calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot, kx, kv);

    const Eigen::Vector3d& f = so3ControlPtr_->getF();
    const Eigen::Quaterniond& q = so3ControlPtr_->getQ();

    so3cmd_.header.stamp = now();
    so3cmd_.force.x = f(0);
    so3cmd_.force.y = f(1);
    so3cmd_.force.z = f(2);
    so3cmd_.orientation.x = q.x();
    so3cmd_.orientation.y = q.y();
    so3cmd_.orientation.z = q.z();
    so3cmd_.orientation.w = q.w();
    so3cmd_pub_->publish(so3cmd_);

    // store last des_pos and des_yaw
    des_pos_ = des_pos;
    des_yaw_ = des_yaw;
  }

  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    so3ControlPtr_->setPos(pos);
    so3ControlPtr_->setVel(vel);

    so3cmd_.aux.current_yaw = tf2::getYaw(msg->pose.pose.orientation);
    des_pos_ = pos;
    des_yaw_ = so3cmd_.aux.current_yaw;
  }

  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    so3ControlPtr_->setAcc(acc);
  }

  std::shared_ptr<SO3Controller> so3ControlPtr_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3cmd_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  quadrotor_msgs::msg::SO3Command so3cmd_{};

  bool position_cmd_received_flag_{false};
  Eigen::Vector3d des_pos_{Eigen::Vector3d::Zero()};
  double des_yaw_{0.0};
};

}  // namespace so3_controller

RCLCPP_COMPONENTS_REGISTER_NODE(so3_controller::SO3ControllerComponent)

