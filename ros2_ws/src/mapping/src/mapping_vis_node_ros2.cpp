#include <rclcpp/rclcpp.hpp>

#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cstdint>

namespace mapping_ros2 {

static sensor_msgs::msg::PointCloud2 occMapToPointCloud2(const quadrotor_msgs::msg::OccMap3d& msg,
                                                         bool remove_floor_ceil,
                                                         double floor_z,
                                                         double ceil_z) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(static_cast<size_t>(msg.data.size() / 8));

  const double res = msg.resolution;
  const int sx = msg.size_x;
  const int sy = msg.size_y;
  const int sz = msg.size_z;
  const int ox = msg.offset_x;
  const int oy = msg.offset_y;
  const int oz = msg.offset_z;

  // data layout is (z * sy + y) * sx + x
  const auto idx = [sx, sy](int x, int y, int z) { return (z * sy + y) * sx + x; };

  for (int x = 0; x < sx; ++x) {
    for (int y = 0; y < sy; ++y) {
      for (int z = 0; z < sz; ++z) {
        const int i = idx(x, y, z);
        if (i < 0 || i >= static_cast<int>(msg.data.size())) {
          continue;
        }
        if (msg.data[static_cast<size_t>(i)] != 1) {  // occupied
          continue;
        }
        const double px = (ox + x + 0.5) * res;
        const double py = (oy + y + 0.5) * res;
        const double pz = (oz + z + 0.5) * res;
        if (remove_floor_ceil) {
          if (!(pz > floor_z && pz < ceil_z)) {
            continue;
          }
        }
        cloud.points.emplace_back(static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz));
      }
    }
  }
  cloud.width = static_cast<uint32_t>(cloud.points.size());
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(cloud, out);
  out.header.frame_id = "world";
  return out;
}

class MappingVisNode : public rclcpp::Node {
 public:
  MappingVisNode() : rclcpp::Node("mapping_vis") {
    remove_floor_ceil_ = declare_parameter<bool>("remove_floor_ceil", false);
    floor_z_ = declare_parameter<double>("floor_z", 0.5);
    ceil_z_ = declare_parameter<double>("ceil_z", 2.5);

    vs_gridmap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("vs_gridmap", 1);
    vs_gridmap_inflate_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("vs_gridmap_inflate", 1);

    gridmap_sub_ = create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap", 1, std::bind(&MappingVisNode::onGridmap, this, std::placeholders::_1));
    gridmap_inflate_sub_ = create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap_inflate", 1, std::bind(&MappingVisNode::onGridmapInflate, this, std::placeholders::_1));
  }

 private:
  void onGridmap(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr msg) {
    auto pc = occMapToPointCloud2(*msg, false, floor_z_, ceil_z_);
    pc.header.stamp = now();
    vs_gridmap_pub_->publish(pc);
  }

  void onGridmapInflate(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr msg) {
    auto pc = occMapToPointCloud2(*msg, remove_floor_ceil_, floor_z_, ceil_z_);
    pc.header.stamp = now();
    vs_gridmap_inflate_pub_->publish(pc);
  }

  bool remove_floor_ceil_{false};
  double floor_z_{0.5};
  double ceil_z_{2.5};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vs_gridmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vs_gridmap_inflate_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_inflate_sub_;
};

}  // namespace mapping_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapping_ros2::MappingVisNode>());
  rclcpp::shutdown();
  return 0;
}

