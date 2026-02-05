#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace odom_visualization_ros2 {

class OdomVisualizationCarNode : public rclcpp::Node {
 public:
  OdomVisualizationCarNode() : rclcpp::Node("odom_visualization_car") {
    mesh_resource_ = declare_parameter<std::string>(
      "mesh_resource", "package://odom_visualization/meshes/car.dae");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    robot_scale_ = declare_parameter<double>("robot_scale", 1.0);

    color_r_ = declare_parameter<double>("color.r", 0.2);
    color_g_ = declare_parameter<double>("color.g", 0.8);
    color_b_ = declare_parameter<double>("color.b", 1.0);
    color_a_ = declare_parameter<double>("color.a", 1.0);
    max_path_len_ = declare_parameter<int>("max_path_len", 2000);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_msg_.header.frame_id = frame_id_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&OdomVisualizationCarNode::onOdom, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "cmd", 10,
      std::bind(&OdomVisualizationCarNode::onCmd, this, std::placeholders::_1));
  }

 private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto stamp = msg->header.stamp;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;
    pose.pose = msg->pose.pose;
    pose_pub_->publish(pose);

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose);
    if (static_cast<int>(path_msg_.poses.size()) > max_path_len_) {
      path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + (path_msg_.poses.size() - max_path_len_));
    }
    path_pub_->publish(path_msg_);

    publishMesh(pose);
    publishTf(pose);
  }

  void onCmd(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr /*msg*/) {}

  void publishMesh(const geometry_msgs::msg::PoseStamped& pose) {
    visualization_msgs::msg::Marker m;
    m.header = pose.header;
    m.ns = "robot";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.mesh_resource = mesh_resource_;
    m.pose = pose.pose;
    m.scale.x = robot_scale_;
    m.scale.y = robot_scale_;
    m.scale.z = robot_scale_;
    m.color.r = static_cast<float>(color_r_);
    m.color.g = static_cast<float>(color_g_);
    m.color.b = static_cast<float>(color_b_);
    m.color.a = static_cast<float>(color_a_);
    mesh_pub_->publish(m);
  }

  void publishTf(const geometry_msgs::msg::PoseStamped& pose) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose.header;
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = child_frame_id_;
    tf.transform.translation.x = pose.pose.position.x;
    tf.transform.translation.y = pose.pose.position.y;
    tf.transform.translation.z = pose.pose.position.z;
    tf.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  std::string mesh_resource_;
  std::string frame_id_;
  std::string child_frame_id_;
  double robot_scale_{1.0};
  double color_r_{0.2}, color_g_{0.8}, color_b_{1.0}, color_a_{1.0};
  int max_path_len_{2000};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::Path path_msg_;
};

}  // namespace odom_visualization_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<odom_visualization_ros2::OdomVisualizationCarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

