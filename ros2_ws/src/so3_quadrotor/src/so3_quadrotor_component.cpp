#include <so3_quadrotor/quadrotor_dynamics.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace so3_quadrotor {

class So3QuadrotorComponent : public rclcpp::Node {
 public:
  explicit So3QuadrotorComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("so3_quadrotor", options) {
    // parameters
    const double init_x = declare_parameter<double>("init_x", 0.0);
    const double init_y = declare_parameter<double>("init_y", 0.0);
    const double init_z = declare_parameter<double>("init_z", 0.0);
    const double init_yaw = declare_parameter<double>("init_yaw", 0.0);

    Config config;
    config.g = declare_parameter<double>("g", 9.81);
    config.mass = declare_parameter<double>("mass", 1.0);
    const double Ixx = declare_parameter<double>("Ixx", 0.0049);
    const double Iyy = declare_parameter<double>("Iyy", 0.0049);
    const double Izz = declare_parameter<double>("Izz", 0.0088);
    config.J = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();
    config.kf = declare_parameter<double>("kf", 1e-6);
    const double prop_radius = declare_parameter<double>("prop_radius", 0.1);
    config.km = 0.07 * (3 * prop_radius) * config.kf;
    config.arm_length = declare_parameter<double>("arm_length", 0.2);
    config.motor_time_constant = declare_parameter<double>("motor_time_constant", 0.02);
    config.max_rpm = declare_parameter<double>("max_rpm", 25000.0);
    config.min_rpm = declare_parameter<double>("min_rpm", 0.0);

    simulation_rate_ = declare_parameter<int>("simulation_rate", 1000);
    odom_rate_ = declare_parameter<int>("odom_rate", 400);
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");

    quadrotorPtr_ = std::make_shared<Quadrotor>(config);
    quadrotorPtr_->setPos(Eigen::Vector3d(init_x, init_y, init_z));
    quadrotorPtr_->setYpr(Eigen::Vector3d(init_yaw, 0, 0));
    const double rpm = sqrt(config.mass * config.g / 4 / config.kf);
    quadrotorPtr_->setRpm(Eigen::Vector4d(rpm, rpm, rpm, rpm));

    Eigen::Quaterniond quat = uav_utils::ypr_to_quaternion(Eigen::Vector3d(init_yaw, 0, 0));
    cmd_.force[2] = config.mass * config.g;
    cmd_.qw = quat.w();
    cmd_.qx = quat.x();
    cmd_.qy = quat.y();
    cmd_.qz = quat.z();

    // pubs/subs
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vis", 10);
    cmd_sub_ = create_subscription<quadrotor_msgs::msg::SO3Command>(
      "so3cmd", 10,
      std::bind(&So3QuadrotorComponent::cmdCb, this, std::placeholders::_1));

    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_msg_.header.frame_id = world_frame_;
    imu_msg_.header.frame_id = world_frame_;

    initMarkers(prop_radius, config.arm_length);

    next_odom_pub_time_ = now();
    simulation_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / static_cast<double>(simulation_rate_)),
      std::bind(&So3QuadrotorComponent::timerCb, this));
  }

 private:
  void cmdCb(const quadrotor_msgs::msg::SO3Command::ConstSharedPtr cmd_msg) {
    cmd_.force[0] = static_cast<float>(cmd_msg->force.x);
    cmd_.force[1] = static_cast<float>(cmd_msg->force.y);
    cmd_.force[2] = static_cast<float>(cmd_msg->force.z);
    cmd_.qx = static_cast<float>(cmd_msg->orientation.x);
    cmd_.qy = static_cast<float>(cmd_msg->orientation.y);
    cmd_.qz = static_cast<float>(cmd_msg->orientation.z);
    cmd_.qw = static_cast<float>(cmd_msg->orientation.w);
    cmd_.kR[0] = static_cast<float>(cmd_msg->k_r[0]);
    cmd_.kR[1] = static_cast<float>(cmd_msg->k_r[1]);
    cmd_.kR[2] = static_cast<float>(cmd_msg->k_r[2]);
    cmd_.kOm[0] = static_cast<float>(cmd_msg->k_om[0]);
    cmd_.kOm[1] = static_cast<float>(cmd_msg->k_om[1]);
    cmd_.kOm[2] = static_cast<float>(cmd_msg->k_om[2]);
    cmd_.corrections[0] = static_cast<float>(cmd_msg->aux.kf_correction);
    cmd_.corrections[1] = static_cast<float>(cmd_msg->aux.angle_corrections[0]);
    cmd_.corrections[2] = static_cast<float>(cmd_msg->aux.angle_corrections[1]);
    cmd_.current_yaw = static_cast<float>(cmd_msg->aux.current_yaw);
    cmd_.use_external_yaw = cmd_msg->aux.use_external_yaw;
  }

  void timerCb() {
    const auto last_control = control_;
    control_ = quadrotorPtr_->getControl(cmd_);
    for (size_t i = 0; i < 4; ++i) {
      if (std::isnan(control_.rpm[i])) control_.rpm[i] = last_control.rpm[i];
    }

    quadrotorPtr_->setInput(control_.rpm[0], control_.rpm[1], control_.rpm[2], control_.rpm[3]);
    quadrotorPtr_->step(1.0 / static_cast<double>(simulation_rate_));

    const auto tnow = now();
    if (tnow < next_odom_pub_time_) return;
    next_odom_pub_time_ = next_odom_pub_time_ + rclcpp::Duration::from_seconds(1.0 / static_cast<double>(odom_rate_));

    const Eigen::Vector3d& pos = quadrotorPtr_->getPos();
    const Eigen::Vector3d vel = quadrotorPtr_->getVel();
    const Eigen::Vector3d acc = quadrotorPtr_->getAcc();
    const Eigen::Quaterniond quat = quadrotorPtr_->getQuat();
    const Eigen::Vector3d omega = quadrotorPtr_->getOmega();

    publishTf(pos, quat, tnow);
    publishOdom(pos, vel, omega, quat, tnow);
    publishImu(acc, omega, quat, tnow);
    publishMarkers(pos, quat, tnow);
  }

  void publishTf(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat, const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = body_frame_;
    tf.transform.translation.x = pos.x();
    tf.transform.translation.y = pos.y();
    tf.transform.translation.z = pos.z();
    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();
    tf_br_->sendTransform(tf);
  }

  void publishOdom(const Eigen::Vector3d& pos,
                   const Eigen::Vector3d& vel,
                   const Eigen::Vector3d& omega,
                   const Eigen::Quaterniond& quat,
                   const rclcpp::Time& stamp) {
    odom_msg_.header.stamp = stamp;
    odom_msg_.pose.pose.position.x = pos.x();
    odom_msg_.pose.pose.position.y = pos.y();
    odom_msg_.pose.pose.position.z = pos.z();
    odom_msg_.pose.pose.orientation.x = quat.x();
    odom_msg_.pose.pose.orientation.y = quat.y();
    odom_msg_.pose.pose.orientation.z = quat.z();
    odom_msg_.pose.pose.orientation.w = quat.w();
    odom_msg_.twist.twist.linear.x = vel.x();
    odom_msg_.twist.twist.linear.y = vel.y();
    odom_msg_.twist.twist.linear.z = vel.z();
    odom_msg_.twist.twist.angular.x = omega.x();
    odom_msg_.twist.twist.angular.y = omega.y();
    odom_msg_.twist.twist.angular.z = omega.z();
    odom_pub_->publish(odom_msg_);
  }

  void publishImu(const Eigen::Vector3d& acc,
                  const Eigen::Vector3d& omega,
                  const Eigen::Quaterniond& quat,
                  const rclcpp::Time& stamp) {
    imu_msg_.header.stamp = stamp;
    imu_msg_.orientation.x = quat.x();
    imu_msg_.orientation.y = quat.y();
    imu_msg_.orientation.z = quat.z();
    imu_msg_.orientation.w = quat.w();
    imu_msg_.angular_velocity.x = omega.x();
    imu_msg_.angular_velocity.y = omega.y();
    imu_msg_.angular_velocity.z = omega.z();
    imu_msg_.linear_acceleration.x = acc.x();
    imu_msg_.linear_acceleration.y = acc.y();
    imu_msg_.linear_acceleration.z = acc.z();
    imu_pub_->publish(imu_msg_);
  }

  void initMarkers(double prop_radius, double arm_length) {
    vis_msg_.markers.resize(4);
    visualization_msgs::msg::Marker propeller;
    propeller.header.frame_id = world_frame_;
    propeller.type = visualization_msgs::msg::Marker::CYLINDER;
    propeller.action = visualization_msgs::msg::Marker::ADD;
    propeller.scale.x = 2 * prop_radius;
    propeller.scale.y = 0.1 * prop_radius;
    propeller.scale.z = 0.05 * prop_radius;
    propeller.color.a = 1.0f;
    propeller.color.r = 1.0f;
    propeller.color.g = 0.5f;
    propeller.color.b = 0.0f;
    for (size_t i = 0; i < 4; ++i) {
      vis_msg_.markers[i] = propeller;
      vis_msg_.markers[i].id = static_cast<int>(i);
    }

    visualization_msgs::msg::Marker drone_body = propeller;
    drone_body.type = visualization_msgs::msg::Marker::SPHERE;
    drone_body.color.g = 0.7f;
    drone_body.scale.x = arm_length;
    drone_body.scale.y = 0.1 * arm_length;
    drone_body.scale.z = 0.05 * arm_length;
    vis_msg_.markers.push_back(drone_body);
    drone_body.scale.y = arm_length;
    drone_body.scale.x = 0.1 * arm_length;
    vis_msg_.markers.push_back(drone_body);
  }

  void publishMarkers(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat, const rclcpp::Time& stamp) {
    vis_msg_.markers[0].header.stamp = stamp;
    for (auto& m : vis_msg_.markers) {
      m.header.stamp = stamp;
      m.header.frame_id = world_frame_;
    }

    std::vector<Eigen::Vector3d> propellers;
    propellers.emplace_back(0, +quadrotorPtr_->config.arm_length / 2, 0.02);
    propellers.emplace_back(0, -quadrotorPtr_->config.arm_length / 2, 0.02);
    propellers.emplace_back(-quadrotorPtr_->config.arm_length / 2, 0, 0.02);
    propellers.emplace_back(+quadrotorPtr_->config.arm_length / 2, 0, 0.02);

    for (size_t i = 0; i < 4; ++i) {
      const double rpm = quadrotorPtr_->state.motor_rpm.coeff(i);
      if (i / 2) {
        motor_yaw_[i] += rpm * 60.0 / static_cast<double>(odom_rate_);
      } else {
        motor_yaw_[i] -= rpm * 60.0 / static_cast<double>(odom_rate_);
      }
      motor_yaw_[i] = std::fmod(motor_yaw_[i], 2 * M_PI);
      Eigen::Quaterniond quat_propeller =
        quat * uav_utils::ypr_to_quaternion(Eigen::Vector3d(motor_yaw_[i], 0, 0));
      Eigen::Vector3d pos_propeller = pos + quat.toRotationMatrix() * propellers[i];

      vis_msg_.markers[i].pose.position.x = pos_propeller(0);
      vis_msg_.markers[i].pose.position.y = pos_propeller(1);
      vis_msg_.markers[i].pose.position.z = pos_propeller(2);
      vis_msg_.markers[i].pose.orientation.x = quat_propeller.x();
      vis_msg_.markers[i].pose.orientation.y = quat_propeller.y();
      vis_msg_.markers[i].pose.orientation.z = quat_propeller.z();
      vis_msg_.markers[i].pose.orientation.w = quat_propeller.w();
    }
    for (size_t i = 4; i < vis_msg_.markers.size(); ++i) {
      vis_msg_.markers[i].pose = odom_msg_.pose.pose;
      vis_msg_.markers[i].id = static_cast<int>(i);
    }
    vis_pub_->publish(vis_msg_);
  }

  std::shared_ptr<Quadrotor> quadrotorPtr_;
  Control control_{};
  Cmd cmd_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  int simulation_rate_{1000};
  int odom_rate_{400};
  std::string world_frame_{"world"};
  std::string body_frame_{"body"};

  nav_msgs::msg::Odometry odom_msg_{};
  sensor_msgs::msg::Imu imu_msg_{};
  visualization_msgs::msg::MarkerArray vis_msg_{};
  double motor_yaw_[4] = {0, 0, 0, 0};

  rclcpp::Time next_odom_pub_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace so3_quadrotor

RCLCPP_COMPONENTS_REGISTER_NODE(so3_quadrotor::So3QuadrotorComponent)

