#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <string>
#include <unordered_map>
#include <vector>

namespace visualization_ros2 {

enum class Color { white, red, green, blue, yellow, greenblue };

inline void setMarkerColor(visualization_msgs::msg::Marker& marker, Color color, double a = 1.0) {
  marker.color.a = static_cast<float>(a);
  switch (color) {
    case Color::white: marker.color.r = 1; marker.color.g = 1; marker.color.b = 1; break;
    case Color::red: marker.color.r = 1; marker.color.g = 0; marker.color.b = 0; break;
    case Color::green: marker.color.r = 0; marker.color.g = 1; marker.color.b = 0; break;
    case Color::blue: marker.color.r = 0; marker.color.g = 0; marker.color.b = 1; break;
    case Color::yellow: marker.color.r = 1; marker.color.g = 1; marker.color.b = 0; break;
    case Color::greenblue: marker.color.r = 0; marker.color.g = 1; marker.color.b = 1; break;
  }
}

inline void setMarkerScale(visualization_msgs::msg::Marker& marker, double x, double y, double z) {
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
}

inline void setMarkerPose(visualization_msgs::msg::Marker& marker, double x, double y, double z) {
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
}

class Visualization {
 public:
  explicit Visualization(rclcpp::Node* node, std::string frame_id = "world")
  : node_(node), frame_id_(std::move(frame_id)) {}

  void visualize_path(const std::vector<Eigen::Vector3d>& path, const std::string& topic) {
    auto pub = getPathPub(topic);
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = frame_id_;
    path_msg.header.stamp = node_->now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    for (const auto& p : path) {
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    pub->publish(path_msg);
  }

  void visualize_pointcloud(const std::vector<Eigen::Vector3d>& pts, const std::string& topic) {
    // Use Marker POINTS to avoid PCL dependency in planning.
    auto pub = getMarkerPub(topic);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node_->now();
    m.ns = topic;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    setMarkerPose(m, 0, 0, 0);
    setMarkerScale(m, 0.05, 0.05, 0.05);
    setMarkerColor(m, Color::yellow, 1.0);
    m.points.reserve(pts.size());
    geometry_msgs::msg::Point p;
    for (const auto& v : pts) {
      p.x = v.x();
      p.y = v.y();
      p.z = v.z();
      m.points.push_back(p);
    }
    pub->publish(m);
  }

  void visualize_arrow(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const std::string& topic, Color color = Color::blue) {
    auto pub = getMarkerPub(topic);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node_->now();
    m.ns = topic;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    setMarkerPose(m, 0, 0, 0);
    setMarkerScale(m, 0.05, 0.1, 0.0);
    setMarkerColor(m, color, 1.0);
    m.points.resize(2);
    m.points[0].x = p0.x(); m.points[0].y = p0.y(); m.points[0].z = p0.z();
    m.points[1].x = p1.x(); m.points[1].y = p1.y(); m.points[1].z = p1.z();
    pub->publish(m);
  }

  // eg for PAIRLINE: std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  void visualize_pairline(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& pairline,
                          const std::string& topic,
                          Color color = Color::greenblue) {
    auto pub = getMarkerPub(topic);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node_->now();
    m.ns = topic;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    setMarkerPose(m, 0, 0, 0);
    setMarkerScale(m, 0.02, 0.02, 0.02);
    setMarkerColor(m, color, 1.0);
    m.points.reserve(pairline.size() * 2);
    geometry_msgs::msg::Point p;
    for (const auto& seg : pairline) {
      p.x = seg.first.x(); p.y = seg.first.y(); p.z = seg.first.z();
      m.points.push_back(p);
      p.x = seg.second.x(); p.y = seg.second.y(); p.z = seg.second.z();
      m.points.push_back(p);
    }
    pub->publish(m);
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr getPathPub(const std::string& topic) {
    auto it = path_pubs_.find(topic);
    if (it != path_pubs_.end()) return it->second;
    auto pub = node_->create_publisher<nav_msgs::msg::Path>(topic, 10);
    path_pubs_[topic] = pub;
    return pub;
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr getMarkerPub(const std::string& topic) {
    auto it = marker_pubs_.find(topic);
    if (it != marker_pubs_.end()) return it->second;
    auto pub = node_->create_publisher<visualization_msgs::msg::Marker>(topic, 10);
    marker_pubs_[topic] = pub;
    return pub;
  }

  rclcpp::Node* node_{nullptr};
  std::string frame_id_;
  std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_pubs_;
};

}  // namespace visualization_ros2

