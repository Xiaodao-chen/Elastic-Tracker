#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud;
ros::Publisher pub_depth;
ros::Publisher pub_pose;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
int cam_width, cam_height;
double cam_fx, cam_fy, cam_cx, cam_cy;

// Camera to body transformation (same as CUDA version)
Eigen::Matrix4d cam02body;
Eigen::Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;

ros::Time last_odom_stamp = ros::TIME_MAX;
Eigen::Vector3d last_pose_world;

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  has_odom = true;
  _odom = odom;
  
  Matrix4d Pose_receive = Matrix4d::Identity();
  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  // Convert to camera pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);

  last_odom_stamp = odom.header.stamp;
  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;

  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  // Create depth image
  cv::Mat depth_image = cv::Mat::zeros(cam_height, cam_width, CV_32FC1);
  
  // Get camera pose (world to camera transformation)
  Eigen::Matrix4d cam_pose = cam2world.inverse();
  Eigen::Matrix3d R_cam = cam_pose.block<3, 3>(0, 0);
  Eigen::Vector3d t_cam = cam_pose.block<3, 1>(0, 3);

  pcl::PointXYZ pt;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // Check vertical FOV
      if ((fabs(pt.z - _odom.pose.pose.position.z) / (sensing_horizon)) >
          tan(M_PI / 12.0))
        continue;

      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      // Check if point is in front of the robot
      if (pt_vec.dot(yaw_vec) < 0) continue;

      _local_map.points.push_back(pt);

      // Transform point to camera frame
      Eigen::Vector3d pt_world(pt.x, pt.y, pt.z);
      Eigen::Vector3d pt_cam_frame = R_cam * pt_world + t_cam;

      // Skip points behind camera
      if (pt_cam_frame.z() <= 0.1) continue;

      // Project to image plane
      double u = cam_fx * pt_cam_frame.x() / pt_cam_frame.z() + cam_cx;
      double v = cam_fy * pt_cam_frame.y() / pt_cam_frame.z() + cam_cy;

      // Check if point is within image bounds
      if (u < 0 || u >= cam_width || v < 0 || v >= cam_height) continue;

      int iu = static_cast<int>(u + 0.5);
      int iv = static_cast<int>(v + 0.5);
      
      if (iu < 0 || iu >= cam_width || iv < 0 || iv >= cam_height) continue;

      float depth = static_cast<float>(pt_cam_frame.z());

      // Fill depth with a small radius for better coverage
      int radius = static_cast<int>(0.05 * cam_fx / depth + 0.5);
      radius = std::max(1, std::min(radius, 5));
      
      for (int di = -radius; di <= radius; di++) {
        for (int dj = -radius; dj <= radius; dj++) {
          int ni = iv + di;
          int nj = iu + dj;
          if (ni >= 0 && ni < cam_height && nj >= 0 && nj < cam_width) {
            float& current_depth = depth_image.at<float>(ni, nj);
            if (current_depth == 0 || depth < current_depth) {
              current_depth = depth;
            }
          }
        }
      }
    }
  } else {
    return;
  }

  // Publish point cloud
  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "world";
  _local_map_pcd.header.stamp = last_odom_stamp;

  pub_cloud.publish(_local_map_pcd);

  // Publish depth image
  cv_bridge::CvImage depth_msg;
  depth_msg.header.stamp = last_odom_stamp;
  depth_msg.header.frame_id = "camera";
  depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg.image = depth_image;
  pub_depth.publish(depth_msg.toImageMsg());

  // Publish camera pose
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "world";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose.publish(camera_pose);
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);
  
  // Get camera parameters
  nh.param("cam_width", cam_width, 640);
  nh.param("cam_height", cam_height, 480);
  nh.param("cam_fx", cam_fx, 387.229248046875);
  nh.param("cam_fy", cam_fy, 387.229248046875);
  nh.param("cam_cx", cam_cx, 321.04638671875);
  nh.param("cam_cy", cam_cy, 243.44969177246094);

  // Camera to body transformation (same as CUDA version)
  cam02body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  cam2world = Eigen::Matrix4d::Identity();

  // Subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // Publisher depth image and point cloud
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 10);
  pub_depth = nh.advertise<sensor_msgs::Image>("depth", 10);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("camera_pose", 10);

  double sensing_duration = 1.0 / sensing_rate * 2.5;

  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  ROS_INFO("Non-CUDA depth render node started (cam: %dx%d, fx=%.2f, fy=%.2f)", 
           cam_width, cam_height, cam_fx, cam_fy);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
