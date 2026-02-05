#include <mapping_core/mapping.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <atomic>
#include <thread>

namespace mapping {

using ImageMsg = sensor_msgs::msg::Image;
using OdomMsg = nav_msgs::msg::Odometry;
using OccMapMsg = quadrotor_msgs::msg::OccMap3d;

using ImageOdomSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, OdomMsg>;
using ImageOdomSynchronizer = message_filters::Synchronizer<ImageOdomSyncPolicy>;

struct CamConfig {
  double rate{0.0};
  double range{0.0};
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double depth_scaling_factor{1000.0};
};

class MappingComponent : public rclcpp::Node {
 public:
  explicit MappingComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mapping", options) {
    declareAllParams();

    gridmap_inflate_pub_ = create_publisher<OccMapMsg>("gridmap_inflate", 1);
    local_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_pointcloud", 1);
    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("mask_cloud", 10);

    // defer message_filters init until we can safely use shared_from_this()
    start_timer_ = create_wall_timer(std::chrono::milliseconds(0), [this]() {
      start_timer_->cancel();
      start();
    });
  }

 private:
  void declareAllParams() {
    declare_parameter<std::vector<double>>("cam2body_R", std::vector<double>());
    declare_parameter<std::vector<double>>("cam2body_p", std::vector<double>());

    use_global_map_ = declare_parameter<bool>("use_global_map", false);
    use_mask_ = declare_parameter<bool>("use_mask", false);

    // global map
    declare_parameter<double>("x_length", 20.0);
    declare_parameter<double>("y_length", 20.0);
    declare_parameter<double>("z_length", 5.0);

    // camera + local map
    camConfig_.rate = declare_parameter<double>("camera_rate", 30.0);
    camConfig_.range = declare_parameter<double>("camera_range", 5.0);
    camConfig_.width = declare_parameter<int>("cam_width", 640);
    camConfig_.height = declare_parameter<int>("cam_height", 480);
    camConfig_.fx = declare_parameter<double>("cam_fx", 525.0);
    camConfig_.fy = declare_parameter<double>("cam_fy", 525.0);
    camConfig_.cx = declare_parameter<double>("cam_cx", 319.5);
    camConfig_.cy = declare_parameter<double>("cam_cy", 239.5);
    camConfig_.depth_scaling_factor = declare_parameter<double>("depth_scaling_factor", 1000.0);

    down_sample_factor_ = declare_parameter<int>("down_sample_factor", 2);
    inflate_size_ = declare_parameter<int>("inflate_size", 2);
    resolution_ = declare_parameter<double>("resolution", 0.1);
    local_x_ = declare_parameter<double>("local_x", 10.0);
    local_y_ = declare_parameter<double>("local_y", 10.0);
    local_z_ = declare_parameter<double>("local_z", 3.0);

    depth_filter_tolerance_ = declare_parameter<double>("depth_filter_tolerance", 0.2);
    depth_filter_mindist_ = declare_parameter<double>("depth_filter_mindist", 0.2);
    depth_filter_margin_ = declare_parameter<int>("depth_filter_margin", 2);

    p_min_ = declare_parameter<int>("p_min", -199);
    p_max_ = declare_parameter<int>("p_max", 220);
    p_hit_ = declare_parameter<int>("p_hit", 62);
    p_mis_ = declare_parameter<int>("p_mis", -62);
    p_occ_ = declare_parameter<int>("p_occ", 139);
    p_def_ = declare_parameter<int>("p_def", -199);
  }

  void start() {
    // cam2body extrinsics
    std::vector<double> tmp;
    get_parameter("cam2body_R", tmp);
    if (tmp.size() == 9) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(tmp.data());
    } else {
      cam2body_R_.setIdentity();
    }
    get_parameter("cam2body_p", tmp);
    if (tmp.size() == 3) {
      cam2body_p_ = Eigen::Map<const Eigen::Vector3d>(tmp.data());
    } else {
      cam2body_p_.setZero();
    }

    // setup map
    if (use_global_map_) {
      const double x = get_parameter("x_length").as_double();
      const double y = get_parameter("y_length").as_double();
      const double z = get_parameter("z_length").as_double();
      gridmap_.setup(resolution_, Eigen::Vector3d(x, y, z), 10, true);
    } else {
      gridmap_.setup(resolution_, Eigen::Vector3d(local_x_, local_y_, local_z_), camConfig_.range);
      gridmap_.setupP(p_min_, p_max_, p_hit_, p_mis_, p_occ_, p_def_);
    }
    gridmap_.inflate_size = inflate_size_;

    if (use_global_map_) {
      map_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "global_map", 1, std::bind(&MappingComponent::mapCb, this, std::placeholders::_1));
      global_map_timer_ = create_wall_timer(std::chrono::seconds(1),
                                            std::bind(&MappingComponent::globalMapTimerCb, this));
    } else {
      depth_sub_.subscribe(shared_from_this(), "depth");
      odom_sub_.subscribe(shared_from_this(), "odom");
      depth_odom_sync_ = std::make_shared<ImageOdomSynchronizer>(ImageOdomSyncPolicy(100), depth_sub_, odom_sub_);
      depth_odom_sync_->registerCallback(std::bind(&MappingComponent::depthOdomCb, this,
                                                   std::placeholders::_1, std::placeholders::_2));
    }

    if (use_mask_) {
      target_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "target", rclcpp::SensorDataQoS(),
        std::bind(&MappingComponent::targetOdomCb, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "mapping started (use_global_map=%d use_mask=%d)", (int)use_global_map_, (int)use_mask_);
  }

  void depthOdomCb(const ImageMsg::ConstSharedPtr& depth_msg, const OdomMsg::ConstSharedPtr& odom_msg) {
    if (callback_lock_.test_and_set()) return;

    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);

    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, camConfig_.depth_scaling_factor);
    }
    cv::Mat depth_img = depth_ptr->image;

    const int nr = depth_img.rows;
    const int nc = depth_img.cols;
    std::vector<Eigen::Vector3d> obs_pts;

    for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
      for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
        double z = (depth_img.at<uint16_t>(i, j)) / camConfig_.depth_scaling_factor;
        if (depth_img.at<uint16_t>(i, j) == 0) {
          z = camConfig_.range + 0.5;
        }
        if (std::isnan(z) || std::isinf(z)) continue;
        if (z < depth_filter_mindist_) continue;

        const double y = (i - camConfig_.cy) * z / camConfig_.fy;
        const double x = (j - camConfig_.cx) * z / camConfig_.fx;
        Eigen::Vector3d p(x, y, z);
        p = cam_q * p + cam_p;

        bool good_point = true;
        if (get_first_frame_) {
          Eigen::Vector3d p_rev_proj = last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
          const double vv = p_rev_proj.y() * camConfig_.fy / p_rev_proj.z() + camConfig_.cy;
          const double uu = p_rev_proj.x() * camConfig_.fx / p_rev_proj.z() + camConfig_.cx;
          if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
            const double drift_dis =
              fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / camConfig_.depth_scaling_factor - p_rev_proj.z());
            if (drift_dis > depth_filter_tolerance_) good_point = false;
          }
        }
        if (good_point) obs_pts.push_back(p);
      }
    }

    last_depth_ = depth_img;
    last_cam_p_ = cam_p;
    last_cam_q_ = cam_q;
    get_first_frame_ = true;

    gridmap_.updateMap(cam_p, obs_pts);

    if (use_mask_) {
      while (target_lock_.test_and_set())
        ;
      Eigen::Vector3d ld = target_odom_;
      Eigen::Vector3d ru = target_odom_;
      ld.x() -= 0.5;
      ld.y() -= 0.5;
      ld.z() -= 1.0;
      ru.x() += 0.5;
      ru.y() += 0.5;
      ru.z() += 1.0;
      gridmap_.setFree(ld, ru);
      target_lock_.clear();
    }

    OccMapMsg gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_->publish(gridmap_msg);

    callback_lock_.clear();
  }

  void targetOdomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    while (target_lock_.test_and_set())
      ;
    target_odom_.x() = msg->pose.pose.position.x;
    target_odom_.y() = msg->pose.pose.position.y;
    target_odom_.z() = msg->pose.pose.position.z;
    target_lock_.clear();
  }

  void mapCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (map_received_) return;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);
    for (const auto& pt : point_cloud) {
      gridmap_.setOcc(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    gridmap_.inflate(inflate_size_);
    RCLCPP_WARN(get_logger(), "[mapping] GLOBAL MAP RECEIVED!");
    map_received_ = true;
  }

  void globalMapTimerCb() {
    if (!map_received_) return;
    OccMapMsg gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_->publish(gridmap_msg);
  }

  // params/state
  CamConfig camConfig_;
  int down_sample_factor_{2};
  Eigen::Matrix3d cam2body_R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d cam2body_p_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d last_cam_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_cam_q_{Eigen::Quaterniond::Identity()};
  bool get_first_frame_{false};
  cv::Mat last_depth_;
  double depth_filter_tolerance_{0.2};
  double depth_filter_mindist_{0.2};
  int depth_filter_margin_{2};

  double resolution_{0.1};
  double local_x_{10.0}, local_y_{10.0}, local_z_{3.0};
  int inflate_size_{2};
  int p_min_{-199}, p_max_{220}, p_hit_{62}, p_mis_{-62}, p_occ_{139}, p_def_{-199};

  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  message_filters::Subscriber<OdomMsg> odom_sub_;
  std::shared_ptr<ImageOdomSynchronizer> depth_odom_sync_;

  rclcpp::Publisher<OccMapMsg>::SharedPtr gridmap_inflate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  rclcpp::TimerBase::SharedPtr start_timer_;

  // global map
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_sub_;
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  bool map_received_{false};
  bool use_global_map_{false};

  // mask target
  bool use_mask_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d target_odom_{Eigen::Vector3d::Zero()};

  OccGridMap gridmap_;
};

}  // namespace mapping

RCLCPP_COMPONENTS_REGISTER_NODE(mapping::MappingComponent)

