#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <cmath>
#include <vector>

namespace local_sensing_ros2 {

class PclRenderNode : public rclcpp::Node {
 public:
  PclRenderNode() : rclcpp::Node("pcl_render") {
    // parameters (API align with ROS1)
    sensing_horizon_ = declare_parameter<double>("sensing_horizon", 5.0);
    sensing_rate_ = declare_parameter<double>("sensing_rate", 30.0);
    estimation_rate_ = declare_parameter<double>("estimation_rate", 30.0);

    cam_width_ = declare_parameter<int>("cam_width", 640);
    cam_height_ = declare_parameter<int>("cam_height", 480);
    cam_fx_ = declare_parameter<double>("cam_fx", 387.229248046875);
    cam_fy_ = declare_parameter<double>("cam_fy", 387.229248046875);
    cam_cx_ = declare_parameter<double>("cam_cx", 321.04638671875);
    cam_cy_ = declare_parameter<double>("cam_cy", 243.44969177246094);

    // same as ROS1 non-CUDA implementation
    cam02body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;
    cam2world_ = Eigen::Matrix4d::Identity();

    global_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", 1, std::bind(&PclRenderNode::rcvGlobalPointCloudCallBack, this, std::placeholders::_1));
    local_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "local_map", 1, std::bind(&PclRenderNode::rcvLocalPointCloudCallBack, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 50, std::bind(&PclRenderNode::rcvOdometryCallbck, this, std::placeholders::_1));

    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_cloud", 10);
    pub_depth_ = create_publisher<sensor_msgs::msg::Image>("depth", 10);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);

    const double sensing_duration = 1.0 / sensing_rate_ * 2.5;
    local_sensing_timer_ = create_wall_timer(
      std::chrono::duration<double>(sensing_duration),
      std::bind(&PclRenderNode::renderSensedPoints, this));

    RCLCPP_INFO(get_logger(),
                "Non-CUDA depth render node started (cam: %dx%d, fx=%.2f, fy=%.2f)",
                cam_width_, cam_height_, cam_fx_, cam_fy_);
  }

 private:
  void rcvOdometryCallbck(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    has_odom_ = true;
    odom_ = *odom;

    Eigen::Matrix4d pose_recv = Eigen::Matrix4d::Identity();
    const Eigen::Vector3d p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    const Eigen::Quaterniond q(odom->pose.pose.orientation.w,
                              odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y,
                              odom->pose.pose.orientation.z);
    pose_recv.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    pose_recv.block<3, 1>(0, 3) = p;

    cam2world_ = pose_recv * cam02body_;
    cam2world_quat_ = Eigen::Quaterniond(cam2world_.block<3, 3>(0, 0));

    last_odom_stamp_ = rclcpp::Time(odom->header.stamp);
    last_pose_world_ = p;
  }

  void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (has_global_map_) {
      return;
    }
    RCLCPP_WARN(get_logger(), "Global Pointcloud received..");

    pcl::PointCloud<pcl::PointXYZ> cloud_input;
    pcl::fromROSMsg(*msg, cloud_input);

    voxel_sampler_.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_sampler_.setInputCloud(cloud_input.makeShared());
    voxel_sampler_.filter(cloud_all_map_);

    kdtree_local_map_.setInputCloud(cloud_all_map_.makeShared());
    has_global_map_ = true;
  }

  void rcvLocalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
    // ROS1: do nothing (fix later). Keep aligned.
  }

  void renderSensedPoints() {
    if (!has_global_map_ || !has_odom_) {
      return;
    }

    Eigen::Quaterniond q(odom_.pose.pose.orientation.w,
                         odom_.pose.pose.orientation.x,
                         odom_.pose.pose.orientation.y,
                         odom_.pose.pose.orientation.z);
    Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
    const Eigen::Vector3d yaw_vec = rot.col(0);

    local_map_.points.clear();

    const pcl::PointXYZ search_point(odom_.pose.pose.position.x,
                                     odom_.pose.pose.position.y,
                                     odom_.pose.pose.position.z);
    point_idx_radius_search_.clear();
    point_radius_squared_distance_.clear();

    cv::Mat depth_image = cv::Mat::zeros(cam_height_, cam_width_, CV_32FC1);

    // world -> camera transform
    const Eigen::Matrix4d cam_pose = cam2world_.inverse();
    const Eigen::Matrix3d R_cam = cam_pose.block<3, 3>(0, 0);
    const Eigen::Vector3d t_cam = cam_pose.block<3, 1>(0, 3);

    if (kdtree_local_map_.radiusSearch(search_point, sensing_horizon_,
                                       point_idx_radius_search_,
                                       point_radius_squared_distance_) <= 0) {
      return;
    }

    for (size_t i = 0; i < point_idx_radius_search_.size(); ++i) {
      const pcl::PointXYZ& pt = cloud_all_map_.points[point_idx_radius_search_[i]];

      if ((fabs(pt.z - odom_.pose.pose.position.z) / sensing_horizon_) > tan(M_PI / 12.0)) {
        continue;
      }

      const Eigen::Vector3d pt_vec(pt.x - odom_.pose.pose.position.x,
                                   pt.y - odom_.pose.pose.position.y,
                                   pt.z - odom_.pose.pose.position.z);
      if (pt_vec.dot(yaw_vec) < 0) {
        continue;
      }

      local_map_.points.push_back(pt);

      // project into camera frame
      const Eigen::Vector3d pt_world(pt.x, pt.y, pt.z);
      const Eigen::Vector3d pt_cam = R_cam * pt_world + t_cam;
      if (pt_cam.z() <= 0.1) {
        continue;
      }

      const double u = cam_fx_ * pt_cam.x() / pt_cam.z() + cam_cx_;
      const double v = cam_fy_ * pt_cam.y() / pt_cam.z() + cam_cy_;
      if (u < 0 || u >= cam_width_ || v < 0 || v >= cam_height_) {
        continue;
      }

      const int iu = static_cast<int>(u + 0.5);
      const int iv = static_cast<int>(v + 0.5);
      if (iu < 0 || iu >= cam_width_ || iv < 0 || iv >= cam_height_) {
        continue;
      }

      const float depth = static_cast<float>(pt_cam.z());

      int radius = static_cast<int>(0.05 * cam_fx_ / depth + 0.5);
      radius = std::max(1, std::min(radius, 5));

      for (int di = -radius; di <= radius; di++) {
        for (int dj = -radius; dj <= radius; dj++) {
          const int ni = iv + di;
          const int nj = iu + dj;
          if (ni >= 0 && ni < cam_height_ && nj >= 0 && nj < cam_width_) {
            float& current = depth_image.at<float>(ni, nj);
            if (current == 0.0f || depth < current) {
              current = depth;
            }
          }
        }
      }
    }

    // publish pointcloud
    local_map_.width = static_cast<uint32_t>(local_map_.points.size());
    local_map_.height = 1;
    local_map_.is_dense = true;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(local_map_, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = last_odom_stamp_;
    pub_cloud_->publish(cloud_msg);

    // publish depth image
    cv_bridge::CvImage depth_cv;
    depth_cv.header.stamp = last_odom_stamp_;
    depth_cv.header.frame_id = "camera";
    depth_cv.encoding = "32FC1";
    depth_cv.image = depth_image;
    pub_depth_->publish(*depth_cv.toImageMsg());

    // publish camera pose
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.stamp = last_odom_stamp_;
    camera_pose.header.frame_id = "world";
    camera_pose.pose.position.x = cam2world_(0, 3);
    camera_pose.pose.position.y = cam2world_(1, 3);
    camera_pose.pose.position.z = cam2world_(2, 3);
    camera_pose.pose.orientation.w = cam2world_quat_.w();
    camera_pose.pose.orientation.x = cam2world_quat_.x();
    camera_pose.pose.orientation.y = cam2world_quat_.y();
    camera_pose.pose.orientation.z = cam2world_quat_.z();
    pub_pose_->publish(camera_pose);
  }

  // params
  double sensing_horizon_{5.0};
  double sensing_rate_{30.0};
  double estimation_rate_{30.0};
  int cam_width_{640};
  int cam_height_{480};
  double cam_fx_{387.229248046875};
  double cam_fy_{387.229248046875};
  double cam_cx_{321.04638671875};
  double cam_cy_{243.44969177246094};

  // state
  bool has_global_map_{false};
  bool has_odom_{false};
  nav_msgs::msg::Odometry odom_{};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d last_pose_world_{Eigen::Vector3d::Zero()};

  Eigen::Matrix4d cam02body_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d cam2world_{Eigen::Matrix4d::Identity()};
  Eigen::Quaterniond cam2world_quat_{1.0, 0.0, 0.0, 0.0};

  // map / pcl
  pcl::PointCloud<pcl::PointXYZ> cloud_all_map_;
  pcl::PointCloud<pcl::PointXYZ> local_map_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler_;
  pcl::search::KdTree<pcl::PointXYZ> kdtree_local_map_;
  std::vector<int> point_idx_radius_search_;
  std::vector<float> point_radius_squared_distance_;

  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::TimerBase::SharedPtr local_sensing_timer_;
};

}  // namespace local_sensing_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_sensing_ros2::PclRenderNode>());
  rclcpp::shutdown();
  return 0;
}

