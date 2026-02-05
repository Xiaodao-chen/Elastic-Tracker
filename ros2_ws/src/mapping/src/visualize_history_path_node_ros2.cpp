#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Geometry>

#include <cmath>

namespace mapping_ros2 {

class VisualizeHistoryPathNode : public rclcpp::Node {
 public:
  VisualizeHistoryPathNode() : rclcpp::Node("visualize_history_path") {
    using std::placeholders::_1;

    // API align with ROS1 defaults/topics
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/position_cmd");
    target_odom_topic_ = declare_parameter<std::string>("target_odom_topic", "/object_odom_dtc2brig");

    auto latched = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();

    drone_path_pub_ = create_publisher<nav_msgs::msg::Path>("history_drone_pose", latched);
    target_path_pub_ = create_publisher<nav_msgs::msg::Path>("history_target_pose", latched);
    lines_pub_ = create_publisher<visualization_msgs::msg::Marker>("drone_target_link_line", latched);
    spring_pub_ = create_publisher<nav_msgs::msg::Path>("spring", latched);

    cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      cmd_topic_, 100, std::bind(&VisualizeHistoryPathNode::onCmd, this, _1));
    target_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      target_odom_topic_, 100, std::bind(&VisualizeHistoryPathNode::onTargetOdom, this, _1));

    RCLCPP_INFO(get_logger(), "visualize_history_path node init ok (cmd=%s target_odom=%s)",
                cmd_topic_.c_str(), target_odom_topic_.c_str());
  }

 private:
  void onCmd(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
    drone_path_.header.frame_id = "world";
    drone_path_.header.stamp = now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position = msg->position;
    pose.pose.orientation.w = 1.0;
    drone_path_.poses.push_back(pose);
    drone_path_pub_->publish(drone_path_);
  }

  void onTargetOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    target_path_.header.frame_id = "world";
    target_path_.header.stamp = now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position = msg->pose.pose.position;
    pose.pose.orientation.w = 1.0;
    target_path_.poses.push_back(pose);
    target_path_pub_->publish(target_path_);

    if (drone_path_.poses.empty()) {
      return;
    }

    const Eigen::Vector3d p0(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const auto& last = drone_path_.poses.back().pose.position;
    const Eigen::Vector3d p1(last.x, last.y, last.z);

    // spring path
    {
      const Eigen::Vector3d dp = p1 - p0;
      if (dp.norm() > 1e-6) {
        const Eigen::Vector3d dx = dp.normalized();
        Eigen::Vector3d dy = Eigen::Vector3d(0, 0, 1).cross(dx);
        if (dy.norm() < 1e-6) {
          dy = Eigen::Vector3d(0, 1, 0);
        } else {
          dy.normalize();
        }
        const Eigen::Vector3d dz = dx.cross(dy);

        nav_msgs::msg::Path spring_msg;
        spring_msg.header.frame_id = "world";
        spring_msg.header.stamp = now();
        for (double t = 0; t < 10 * 2 * M_PI; t += 0.1) {
          const double y = 0.2 * cos(t);
          const double z = 0.2 * sin(t);
          const Eigen::Vector3d p = p0 + dp * t / (10 * 2 * M_PI) + z * dz + y * dy;
          geometry_msgs::msg::PoseStamped sp;
          sp.header.frame_id = "world";
          sp.pose.position.x = p.x();
          sp.pose.position.y = p.y();
          sp.pose.position.z = p.z();
          sp.pose.orientation.w = 1.0;
          spring_msg.poses.push_back(sp);
        }
        spring_pub_->publish(spring_msg);
      }
    }

    // throttle line publishing (0.3s)
    const auto t_now = now();
    if ((t_now - t_last_line_pub_).seconds() < 0.3) {
      return;
    }
    t_last_line_pub_ = t_now;

    visualization_msgs::msg::Marker line_list;
    line_list.header.stamp = t_now;
    line_list.header.frame_id = "world";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.ns = "lines";
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.03;
    line_list.color.r = 0.0f;
    line_list.color.g = 1.0f;
    line_list.color.b = 0.1f;
    line_list.color.a = 0.5f;

    geometry_msgs::msg::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    line_list.points.push_back(p);

    p.x = last.x;
    p.y = last.y;
    p.z = last.z;
    line_list.points.push_back(p);

    lines_pub_->publish(line_list);
  }

  std::string cmd_topic_;
  std::string target_odom_topic_;

  nav_msgs::msg::Path drone_path_;
  nav_msgs::msg::Path target_path_;

  rclcpp::Time t_last_line_pub_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr drone_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lines_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spring_pub_;

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
};

}  // namespace mapping_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapping_ros2::VisualizeHistoryPathNode>());
  rclcpp::shutdown();
  return 0;
}

