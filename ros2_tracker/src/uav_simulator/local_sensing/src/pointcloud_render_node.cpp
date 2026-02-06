#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

// ROS2 publishers
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;

sensor_msgs::msg::PointCloud2 local_map_pcl;
sensor_msgs::msg::PointCloud2 local_depth_pcl;

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub;

rclcpp::TimerBase::SharedPtr local_sensing_timer;

bool has_global_map = false;
bool has_local_map = false;
bool has_odom = false;

nav_msgs::msg::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

rclcpp::Time last_odom_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

// Camera intrinsics for depth image rendering
int cam_width = 640, cam_height = 480;
double cam_fx = 387.229, cam_fy = 387.229;
double cam_cx = 321.046, cam_cy = 243.449;

// Camera to body transform (same as pcl_render_node.cpp CUDA version)
Eigen::Matrix4d cam02body;
Eigen::Matrix4d cam2world;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

// 里程计信息回调
void rcvOdometryCallbck(const nav_msgs::msg::Odometry& odom) {
  has_odom = true;
  _odom = odom;
  last_odom_stamp = odom.header.stamp;

  // Compute body pose in world frame
  Eigen::Quaterniond q;
  q.x() = odom.pose.pose.orientation.x;
  q.y() = odom.pose.pose.orientation.y;
  q.z() = odom.pose.pose.orientation.z;
  q.w() = odom.pose.pose.orientation.w;

  Eigen::Matrix4d body_pose = Eigen::Matrix4d::Identity();
  body_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  body_pose(0, 3) = odom.pose.pose.position.x;
  body_pose(1, 3) = odom.pose.pose.position.y;
  body_pose(2, 3) = odom.pose.pose.position.z;

  // cam2world = body_pose * cam02body
  cam2world = body_pose * cam02body;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::msg::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
  if (has_global_map) return;

  RCLCPP_WARN(rclcpp::get_logger("rcvGlobalPointCloudCallBack"), "Global Pointcloud received..");

  // 转换消息信息格式
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(*pointcloud_map, cloud_input);

  // 使用体素滤波对点云降采样
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  // 结果保存在_cloud_all_map中
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(/*const rclcpp::TimerBase event*/) {
  if (!has_global_map || !has_odom) return;

  // 获取无人机姿态
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  // 转换为旋转矩阵
  Eigen::Vector3d yaw_vec = rot.col(0);

  // 清空并初始化点云数据
  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  // 进行半径搜索获取感知范围内的点
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    // 遍历每个搜索到的点
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // 设置最大仰角
      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))
        continue;

      // 检查点是否在无人机视野内
      Eigen::Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                             pt.y - _odom.pose.pose.position.y,
                             pt.z - _odom.pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue;

      _local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  // 设置局部地图属性并发布
  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "map";
  _local_map_pcd.header.stamp = last_odom_stamp;

  pub_cloud->publish(_local_map_pcd);

  // ========== Generate and publish depth image ==========
  if (pub_depth && has_odom) {
    // Compute world-to-camera transform
    Eigen::Matrix4d world2cam = cam2world.inverse();

    // Create depth image (32FC1, in meters)
    cv::Mat depth_mat = cv::Mat::zeros(cam_height, cam_width, CV_32FC1);

    for (size_t i = 0; i < _local_map.points.size(); ++i) {
      const auto& pt = _local_map.points[i];
      Eigen::Vector4d pw(pt.x, pt.y, pt.z, 1.0);

      // Transform to camera frame
      Eigen::Vector4d pc = world2cam * pw;

      // Camera frame: z forward, x right, y down
      double z = pc(2);
      if (z <= 0.01) continue;  // Behind camera

      // Project to image plane
      double u = cam_fx * pc(0) / z + cam_cx;
      double v = cam_fy * pc(1) / z + cam_cy;

      int iu = static_cast<int>(std::round(u));
      int iv = static_cast<int>(std::round(v));

      if (iu < 0 || iu >= cam_width || iv < 0 || iv >= cam_height)
        continue;

      // Z-buffer: keep the closest point for each pixel
      float& cur_depth = depth_mat.at<float>(iv, iu);
      float new_depth = static_cast<float>(z);
      if (cur_depth == 0.0f || new_depth < cur_depth) {
        cur_depth = new_depth;
      }
    }

    // Publish depth image
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = last_odom_stamp;
    out_msg.header.frame_id = "camera";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image = depth_mat;
    pub_depth->publish(*out_msg.toImageMsg());
  }
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = rclcpp::Node::make_shared("pcl_render");

  // 使用 declare_parameter 来声明参数并读取参数
  node->declare_parameter("sensing_horizon", 0.0);
  node->declare_parameter("sensing_rate", 0.0);
  node->declare_parameter("estimation_rate", 0.0);

  node->declare_parameter("map/x_size", 0.0);
  node->declare_parameter("map/y_size", 0.0);
  node->declare_parameter("map/z_size", 0.0);

  // Camera intrinsics (loaded from camera.yaml)
  node->declare_parameter("cam_width", 640);
  node->declare_parameter("cam_height", 480);
  node->declare_parameter("cam_fx", 387.229248046875);
  node->declare_parameter("cam_fy", 387.229248046875);
  node->declare_parameter("cam_cx", 321.04638671875);
  node->declare_parameter("cam_cy", 243.44969177246094);

  node->get_parameter("sensing_horizon", sensing_horizon);
  node->get_parameter("sensing_rate", sensing_rate);
  node->get_parameter("estimation_rate", estimation_rate);
  node->get_parameter("map/x_size", _x_size);
  node->get_parameter("map/y_size", _y_size);
  node->get_parameter("map/z_size", _z_size);
  node->get_parameter("cam_width", cam_width);
  node->get_parameter("cam_height", cam_height);
  node->get_parameter("cam_fx", cam_fx);
  node->get_parameter("cam_fy", cam_fy);
  node->get_parameter("cam_cx", cam_cx);
  node->get_parameter("cam_cy", cam_cy);

  RCLCPP_INFO(node->get_logger(), "Camera: %dx%d, fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
              cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);

  // Initialize cam02body transform (camera -> body)
  // Same as CUDA version: camera z-forward maps to body x-forward
  cam02body << 0.0, 0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0, 0.0,
               0.0, -1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0;
  cam2world = Eigen::Matrix4d::Identity();

  // 订阅点云数据
  global_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 50, rcvOdometryCallbck);

  // 发布者：点云数据和深度图像
  pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/cloud", 10);
  pub_depth = node->create_publisher<sensor_msgs::msg::Image>("depth", 10);

  // 定时器：控制渲染频率
  double sensing_duration = 1.0 / sensing_rate * 2.5;
  local_sensing_timer = node->create_wall_timer(
      std::chrono::duration<double>(sensing_duration), std::bind(&renderSensedPoints));

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = static_cast<int>(_x_size * _inv_resolution);
  _GLY_SIZE = static_cast<int>(_y_size * _inv_resolution);
  _GLZ_SIZE = static_cast<int>(_z_size * _inv_resolution);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(node);  
    status = rclcpp::ok();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}