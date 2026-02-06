#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/replan_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>

#include <decomp_ros_utils/data_ros_utils.h>
#include <traj_opt/poly_traj_utils.hpp>
#include <traj_opt/traj_opt.h>
#include <visualization/visualization_ros2.hpp>
#include <prediction/prediction_ros2.hpp>
#include <env/env_ros2.hpp>
#include <wr_msg/wr_msg_ros2.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <optional>

namespace planning {

class PlanningComponent : public rclcpp::Node {
 public:
  explicit PlanningComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("planning", options) {
    // minimal params (keep names close to ROS1)
    plan_hz_ = declare_parameter<int>("plan_hz", 10);
    tracking_dist_ = declare_parameter<double>("tracking_dist", 2.0);
    tolerance_d_ = declare_parameter<double>("tolerance_d", 0.5);
    debug_ = declare_parameter<bool>("debug", false);
    fake_ = declare_parameter<bool>("fake", false);
    drone_id_ = static_cast<int16_t>(declare_parameter<int>("drone_id", 0));

    // prediction params (aligned with ROS1 names where possible)
    tracking_dur_ = declare_parameter<double>("tracking_dur", 1.0);
    tracking_dt_ = declare_parameter<double>("tracking_dt", 0.1);
    rho_a_ = declare_parameter<double>("prediction/rho_a", 1.0);
    vmax_ = declare_parameter<double>("prediction/vmax", 5.0);
    theta_clearance_ = declare_parameter<double>("theta_clearance", 0.6);

    // API align: ROS1 topic names
    traj_pub_ = create_publisher<quadrotor_msgs::msg::PolyTraj>("trajectory", 1);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("heartbeat", 1);
    replan_state_pub_ = create_publisher<quadrotor_msgs::msg::ReplanState>("replanState", 1);
    inflate_gridmap_pub_ = create_publisher<quadrotor_msgs::msg::OccMap3d>("gridmap_inflate", 10);
    polyhedra_pub_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("planning/polyhedra", 1);

    gridmap_sub_ = create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap_inflate", 1, std::bind(&PlanningComponent::gridmapCb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&PlanningComponent::odomCb, this, std::placeholders::_1));
    // ROS1-like: consume global target odom (from target_ekf_sim).
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/target_ekf_odom", rclcpp::SensorDataQoS(),
      std::bind(&PlanningComponent::targetCb, this, std::placeholders::_1));
    // API align: ROS1 typo'd topic names
    trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/triger", 10, std::bind(&PlanningComponent::triggerCb, this, std::placeholders::_1));
    land_trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/land_triger", 10, std::bind(&PlanningComponent::landTriggerCb, this, std::placeholders::_1));

    vis_ = std::make_shared<visualization_ros2::Visualization>(this, "world");
    predictor_ = std::make_shared<prediction_ros2::Predict>(tracking_dur_, tracking_dt_, rho_a_, vmax_);
    map_ptr_ = std::make_shared<mapping::OccGridMap>();
    env_ = std::make_shared<env_ros2::Env>(map_ptr_, tracking_dist_, tolerance_d_, theta_clearance_);

    // traj_opt params (align with ROS1 traj_opt)
    traj_opt_params_.K = declare_parameter<int>("K", traj_opt_params_.K);
    traj_opt_params_.vmax = declare_parameter<double>("vmax", traj_opt_params_.vmax);
    traj_opt_params_.amax = declare_parameter<double>("amax", traj_opt_params_.amax);
    traj_opt_params_.rhoT = declare_parameter<double>("rhoT", traj_opt_params_.rhoT);
    traj_opt_params_.rhoP = declare_parameter<double>("rhoP", traj_opt_params_.rhoP);
    traj_opt_params_.rhoV = declare_parameter<double>("rhoV", traj_opt_params_.rhoV);
    traj_opt_params_.rhoA = declare_parameter<double>("rhoA", traj_opt_params_.rhoA);
    traj_opt_params_.rhoTracking = declare_parameter<double>("rhoTracking", traj_opt_params_.rhoTracking);
    traj_opt_params_.rhosVisibility = declare_parameter<double>("rhosVisibility", traj_opt_params_.rhosVisibility);
    traj_opt_params_.clearance_d = declare_parameter<double>("clearance_d", traj_opt_params_.clearance_d);
    traj_opt_params_.tolerance_d = tolerance_d_;
    traj_opt_params_.theta_clearance = theta_clearance_;
    traj_opt_params_.tracking_dur = tracking_dur_;
    traj_opt_params_.tracking_dist = tracking_dist_;
    traj_opt_params_.tracking_dt = tracking_dt_;
    traj_opt_.setParams(traj_opt_params_);

    const double hz = std::max(1, plan_hz_);
    plan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&PlanningComponent::planTimerCb, this));

    if (debug_) {
      // NOTE: ROS1 debug file is not compatible. We use ROS2 serialization format file.
      const std::string debug_file = declare_parameter<std::string>("debug_file", "/root/Elastic-Tracker/debug/replan_state_ros2.bin");
      quadrotor_msgs::msg::ReplanState rs{};
      if (wr_msg::readMsgRos2(rs, debug_file)) {
        debug_replan_state_ = rs;
        map_ptr_->from_msg(rs.occmap);
        predictor_->setMap(*map_ptr_);
        inflate_gridmap_pub_->publish(rs.occmap);
        RCLCPP_WARN(get_logger(), "Loaded ROS2 debug replan state from '%s' (state=%d)", debug_file.c_str(), rs.state);
      } else {
        RCLCPP_WARN(get_logger(), "Debug enabled but failed to read ROS2 debug file '%s'", debug_file.c_str());
      }
    }

    replan_stamp_ = now();

    RCLCPP_WARN(get_logger(),
                "Planning node initialized! (ROS2, API-aligned topics: trajectory/replanState/triger/land_triger)");
  }

 private:
  struct PlannedTraj {
    rclcpp::Time start{0, 0, RCL_ROS_TIME};
    Trajectory traj{};
    float yaw{0.0f};
  };

  static double getYawFromQuatWxyz(double w, double x, double y, double z) {
    // yaw (Z) from quaternion (w,x,y,z), assuming ENU world.
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static std::array<double, 6> quinticCoeffs(double p0, double v0, double a0,
                                             double p1, double v1, double a1,
                                             double T) {
    // Solve for p(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5
    // with boundary constraints at t=0 and t=T for position/velocity/acceleration.
    if (T < 1e-3) {
      T = 1e-3;
    }
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    const double c0 = p0;
    const double c1 = v0;
    const double c2 = 0.5 * a0;

    const double b0 = p1 - (c0 + c1 * T + c2 * T2);
    const double b1 = v1 - (c1 + 2.0 * c2 * T);
    const double b2 = a1 - (2.0 * c2);

    // Closed-form solution for c3..c5
    const double c3 = (10.0 * b0 - 4.0 * b1 * T + 0.5 * b2 * T2) / T3;
    const double c4 = (-15.0 * b0 + 7.0 * b1 * T - 1.0 * b2 * T2) / T4;
    const double c5 = (6.0 * b0 - 3.0 * b1 * T + 0.5 * b2 * T2) / T5;

    return {c0, c1, c2, c3, c4, c5};
  }

  static double evalPoly(const std::array<double, 6>& c, double t) {
    double tt = 1.0;
    double out = 0.0;
    for (int i = 0; i < 6; ++i) {
      out += c[i] * tt;
      tt *= t;
    }
    return out;
  }

  static double evalPolyDot(const std::array<double, 6>& c, double t) {
    // derivative
    double out = 0.0;
    double tt = 1.0;
    for (int i = 1; i < 6; ++i) {
      out += static_cast<double>(i) * c[i] * tt;
      tt *= t;
    }
    return out;
  }

  static double evalPolyDDot(const std::array<double, 6>& c, double t) {
    // second derivative
    double out = 0.0;
    double tt = 1.0;
    for (int i = 2; i < 6; ++i) {
      out += static_cast<double>(i * (i - 1)) * c[i] * tt;
      tt *= t;
    }
    return out;
  }

  static Eigen::Vector3d evalPos(const PlannedTraj& tr, double t) {
    const double tt = std::clamp(t, 0.0, tr.traj.getTotalDuration());
    return tr.traj.getPos(tt);
  }
  static Eigen::Vector3d evalVel(const PlannedTraj& tr, double t) {
    const double tt = std::clamp(t, 0.0, tr.traj.getTotalDuration());
    return tr.traj.getVel(tt);
  }
  static Eigen::Vector3d evalAcc(const PlannedTraj& tr, double t) {
    const double tt = std::clamp(t, 0.0, tr.traj.getTotalDuration());
    return tr.traj.getAcc(tt);
  }

  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_msg_ = *msg;
    odom_received_.store(true);
  }

  void targetCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    target_msg_ = *msg;
    target_received_.store(true);
  }

  void gridmapCb(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr msg) {
    map_msg_ = *msg;
    map_received_.store(true);
  }

  void triggerCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_z_ = 0.9;  // align with ROS1 behavior
    triggered_.store(true);
  }

  void landTriggerCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    land_p_.x() = msg->pose.position.x;
    land_p_.y() = msg->pose.position.y;
    land_p_.z() = msg->pose.position.z;
    land_q_.w() = msg->pose.orientation.w;
    land_q_.x() = msg->pose.orientation.x;
    land_q_.y() = msg->pose.orientation.y;
    land_q_.z() = msg->pose.orientation.z;
    land_triggered_.store(true);
  }

  void publishHoverAt(const Eigen::Vector3d& hover_p, const rclcpp::Time& stamp) {
    quadrotor_msgs::msg::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.drone_id = drone_id_;
    traj_msg.hover_p.resize(3);
    traj_msg.hover_p[0] = static_cast<float>(hover_p.x());
    traj_msg.hover_p[1] = static_cast<float>(hover_p.y());
    traj_msg.hover_p[2] = static_cast<float>(hover_p.z());
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_->publish(traj_msg);
  }

  PlannedTraj makeQuinticTraj(const Eigen::Vector3d& p0,
                              const Eigen::Vector3d& v0,
                              const Eigen::Vector3d& a0,
                              const Eigen::Vector3d& p1,
                              const Eigen::Vector3d& v1,
                              const Eigen::Vector3d& a1,
                              float yaw,
                              const rclcpp::Time& start_stamp) const {
    const double dist = (p1 - p0).norm();
    const double vmax = std::max(0.1, vmax_);
    const double T = std::clamp(dist / vmax, 0.5, 6.0);
    PlannedTraj tr;
    tr.start = start_stamp;
    const auto cx = quinticCoeffs(p0.x(), v0.x(), a0.x(), p1.x(), v1.x(), a1.x(), T);
    const auto cy = quinticCoeffs(p0.y(), v0.y(), a0.y(), p1.y(), v1.y(), a1.y(), T);
    const auto cz = quinticCoeffs(p0.z(), v0.z(), a0.z(), p1.z(), v1.z(), a1.z(), T);
    CoefficientMat cMat;
    cMat.row(0) << cx[5], cx[4], cx[3], cx[2], cx[1], cx[0];
    cMat.row(1) << cy[5], cy[4], cy[3], cy[2], cy[1], cy[0];
    cMat.row(2) << cz[5], cz[4], cz[3], cz[2], cz[1], cz[0];
    tr.traj = Trajectory(std::vector<double>{T}, std::vector<CoefficientMat>{cMat});
    tr.yaw = yaw;
    return tr;
  }

  void publishTrajMsg(const PlannedTraj& tr) {
    quadrotor_msgs::msg::PolyTraj msg;
    msg.hover = false;
    msg.drone_id = drone_id_;
    msg.traj_id = traj_id_++;
    msg.start_time = tr.start;
    msg.order = 5;
    msg.yaw = tr.yaw;
    const int N = tr.traj.getPieceNum();
    msg.duration.resize(N);
    msg.coef_x.resize(N * 6);
    msg.coef_y.resize(N * 6);
    msg.coef_z.resize(N * 6);
    for (int i = 0; i < N; ++i) {
      msg.duration[i] = static_cast<float>(tr.traj[i].getDuration());
      const auto& c = tr.traj[i].getCoeffMat();  // cols t^5..t^0
      const int i6 = i * 6;
      for (int k = 0; k < 6; ++k) {
        msg.coef_x[i6 + k] = static_cast<float>(c(0, k));
        msg.coef_y[i6 + k] = static_cast<float>(c(1, k));
        msg.coef_z[i6 + k] = static_cast<float>(c(2, k));
      }
    }
    traj_pub_->publish(msg);
  }

  bool validcheck(const PlannedTraj& tr, const rclcpp::Time& t_start, double check_dur = 1.0) const {
    if (!map_received_.load()) return true;
    const double t0 = std::max(0.0, (now() - t_start).seconds());
    const double delta_t = std::min(check_dur, tr.traj.getTotalDuration());
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      const Eigen::Vector3d p = evalPos(tr, t);
      if (map_ptr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
  }

  void fillIniStateMsg(std::vector<double>& ini_state, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) {
    ini_state.resize(9);
    ini_state[0] = p.x(); ini_state[1] = p.y(); ini_state[2] = p.z();
    ini_state[3] = v.x(); ini_state[4] = v.y(); ini_state[5] = v.z();
    ini_state[6] = a.x(); ini_state[7] = a.y(); ini_state[8] = a.z();
  }

  void planTimerCb() {
    heartbeat_pub_->publish(std_msgs::msg::Empty());

    if (debug_) {
      debugTimerCb();
      return;
    }
    if (fake_) {
      fakeTimerCb();
      return;
    }
    normalTimerCb();
  }

  void normalTimerCb() {
    if (!odom_received_.load() || !map_received_.load()) return;
    if (!triggered_.load() || !target_received_.load()) return;
    static int tick = 0;
    if ((tick++ % 50) == 0) {
      RCLCPP_INFO(get_logger(), "[planner] tick: force_hover=%d wait_hover=%d land=%d",
                  static_cast<int>(force_hover_), static_cast<int>(wait_hover_), static_cast<int>(land_triggered_.load()));
    }

    // odom state
    const Eigen::Vector3d odom_p(odom_msg_.pose.pose.position.x,
                                 odom_msg_.pose.pose.position.y,
                                 odom_msg_.pose.pose.position.z);
    const Eigen::Vector3d odom_v(odom_msg_.twist.twist.linear.x,
                                 odom_msg_.twist.twist.linear.y,
                                 odom_msg_.twist.twist.linear.z);
    const auto& oq = odom_msg_.pose.pose.orientation;
    const double now_yaw = getYawFromQuatWxyz(oq.w, oq.x, oq.y, oq.z);

    // NOTE force-hover: waiting for speed small enough
    if (force_hover_ && odom_v.norm() > 0.1) return;

    // target state
    nav_msgs::msg::Odometry target_msg = target_msg_;
    Eigen::Vector3d target_p(target_msg.pose.pose.position.x,
                             target_msg.pose.pose.position.y,
                             target_msg.pose.pose.position.z);
    Eigen::Vector3d target_v(target_msg.twist.twist.linear.x,
                             target_msg.twist.twist.linear.y,
                             target_msg.twist.twist.linear.z);
    const auto& tq = target_msg.pose.pose.orientation;
    const double target_yaw = getYawFromQuatWxyz(tq.w, tq.x, tq.y, tq.z);

    // landing handling / hover decision
    if (land_triggered_.load()) {
      if ((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2) {
        if (!wait_hover_) {
          publishHoverAt(odom_p, now());
          wait_hover_ = true;
        }
        quadrotor_msgs::msg::ReplanState rs;
        rs.header.stamp = now();
        rs.header.frame_id = "world";
        rs.state = -1;
        rs.target = target_msg;
        rs.occmap = map_msg_;
        rs.replan_stamp = rs.header.stamp;
        replan_state_pub_->publish(rs);
        return;
      }
      // align with ROS1: target_p = target_p + target_q * land_p_
      target_p = target_p + Eigen::Quaterniond(tq.w, tq.x, tq.y, tq.z) * land_p_;
      wait_hover_ = false;
    } else {
      target_p.z() += 1.0;
      const Eigen::Vector3d dp = target_p - odom_p;
      const double desired_yaw = std::atan2(dp.y(), dp.x());
      if (std::fabs(dp.norm() - tracking_dist_) < tolerance_d_ &&
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
          std::fabs(desired_yaw - now_yaw) < 0.5) {
        if (!wait_hover_) {
          publishHoverAt(odom_p, now());
          wait_hover_ = true;
        }
        quadrotor_msgs::msg::ReplanState rs;
        rs.header.stamp = now();
        rs.header.frame_id = "world";
        rs.state = -1;
        rs.target = target_msg;
        rs.occmap = map_msg_;
        rs.replan_stamp = rs.header.stamp;
        replan_state_pub_->publish(rs);
        return;
      }
      wait_hover_ = false;
    }

    // obtain map
    map_ptr_->from_msg(map_msg_);
    predictor_->setMap(*map_ptr_);

    // visualize ray
    vis_->visualize_arrow(odom_p, target_p, "planning/ray",
                          env_->checkRayValid(odom_p, target_p) ? visualization_ros2::Color::yellow
                                                                : visualization_ros2::Color::red);

    // prediction
    std::vector<Eigen::Vector3d> target_predict;
    bool generate_new_traj_success = predictor_->predict(target_p, target_v, target_predict, 0.05);
    if (generate_new_traj_success) {
      vis_->visualize_path(target_predict, "planning/car_predict");
    }
    if (!generate_new_traj_success) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[planner] predict failed");
    }

    // replan state: init state selection
    const rclcpp::Time replan_stamp = now() + rclcpp::Duration::from_seconds(0.03);
    const double replan_t = (replan_stamp - replan_stamp_).seconds();

    Eigen::Vector3d ini_p, ini_v, ini_a;
    if (force_hover_ || !last_traj_.has_value() || replan_t > last_traj_->traj.getTotalDuration()) {
      ini_p = odom_p;
      ini_v = odom_v;
      ini_a.setZero();
    } else {
      ini_p = evalPos(*last_traj_, replan_t);
      ini_v = evalVel(*last_traj_, replan_t);
      ini_a = evalAcc(*last_traj_, replan_t);
    }

    quadrotor_msgs::msg::ReplanState rs;
    rs.header.stamp = now();
    rs.header.frame_id = "world";
    rs.target = target_msg;
    rs.occmap = map_msg_;
    rs.replan_stamp = replan_stamp;
    fillIniStateMsg(rs.ini_state, ini_p, ini_v, ini_a);

    // path searching
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      if (land_triggered_.load()) {
        generate_new_traj_success = env_->short_astar(ini_p, target_p, path);
      } else {
        generate_new_traj_success = env_->findVisiblePath(ini_p, target_predict, way_pts, path);
      }
    }

    // trajectory optimization (ROS1-equivalent pipeline)
    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success && !path.empty()) {
      vis_->visualize_path(path, "planning/astar");

      if (land_triggered_.load()) {
        // ROS1: append prediction points so corridor reaches landing target
        for (const auto& p : target_predict) path.push_back(p);
      } else if (!target_predict.empty() && !way_pts.empty()) {
        // ROS1: visible region generation
        target_predict.pop_back();
        way_pts.pop_back();
        env_->generate_visible_regions(target_predict, way_pts, visible_ps, thetas);
        vis_->visualize_pointcloud(visible_ps, "planning/visible_ps");
        vis_->visualize_pointcloud(way_pts, "planning/way_pts");

        way_pts.insert(way_pts.begin(), ini_p);
        env_->pts2path(way_pts, path);
      }

      // corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      env_->generateSFC(path, 2.0, hPolys, keyPts);
      if (hPolys.empty()) {
        RCLCPP_WARN(get_logger(), "[planner] generateSFC produced 0 polyhedra");
      }
      vis_->visualize_pairline(keyPts, "planning/keyPts");

      // publish corridor polyhedra for RViz2 plugin
      vec_E<Polyhedron3D> decompPolys;
      for (const auto& poly : hPolys) {
        vec_E<Hyperplane3D> hyper_planes;
        hyper_planes.resize(poly.cols());
        for (int i = 0; i < poly.cols(); ++i) {
          hyper_planes[i].n_ = poly.col(i).head(3);
          hyper_planes[i].p_ = poly.col(i).tail(3);
        }
        decompPolys.emplace_back(hyper_planes);
      }
      auto poly_msg = DecompROS::polyhedron_array_to_ros(decompPolys);
      poly_msg.header.frame_id = "world";
      poly_msg.header.stamp = now();
      polyhedra_pub_->publish(poly_msg);

      // trajectory optimization
      Eigen::MatrixXd iniState(3, 3);
      iniState.col(0) = ini_p;
      iniState.col(1) = ini_v;
      iniState.col(2) = ini_a;

      Eigen::MatrixXd finState = Eigen::MatrixXd::Zero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;

      if (land_triggered_.load()) {
        finState.col(0) = target_predict.back();
        generate_new_traj_success = traj_opt_.generate_traj(iniState, finState, target_predict, hPolys, traj);
      } else {
        generate_new_traj_success = traj_opt_.generate_traj(iniState, finState, target_predict, visible_ps, thetas, hPolys, traj);
      }
      if (!generate_new_traj_success) {
        RCLCPP_WARN(get_logger(), "[planner] traj_opt generate_traj failed");
      }
    } else {
      generate_new_traj_success = false;
    }

    PlannedTraj new_traj;
    bool valid = false;
    if (generate_new_traj_success) {
      new_traj.start = replan_stamp;
      new_traj.traj = traj;
      Eigen::Vector3d dp = target_p + target_v * 0.03 - ini_p;
      double yaw = std::atan2(dp.y(), dp.x());
      if (land_triggered_.load()) yaw = target_yaw;
      new_traj.yaw = static_cast<float>(yaw);
      valid = validcheck(new_traj, replan_stamp);
    } else {
      rs.state = -2;
      replan_state_pub_->publish(rs);
    }

    if (valid) {
      force_hover_ = false;
      rs.state = 0;  // REPLAN SUCCESS
      replan_state_pub_->publish(rs);
      publishTrajMsg(new_traj);
      last_traj_ = new_traj;
      replan_stamp_ = replan_stamp;
      return;
    }

    // failure handling (aligned with ROS1 semantics)
    if (force_hover_) {
      rs.state = 1;  // REPLAN FAILED, HOVERING...
      replan_state_pub_->publish(rs);
      return;
    }

    if (last_traj_.has_value() && !validcheck(*last_traj_, replan_stamp_)) {
      force_hover_ = true;
      rs.state = 2;  // EMERGENCY STOP
      replan_state_pub_->publish(rs);
      publishHoverAt(ini_p, replan_stamp);
      return;
    }

    rs.state = 3;  // EXECUTE LAST TRAJ (or do nothing)
    replan_state_pub_->publish(rs);
    if (last_traj_.has_value()) {
      publishTrajMsg(*last_traj_);
    }
  }

  void fakeTimerCb() {
    if (!odom_received_.load() || !map_received_.load()) return;
    if (!triggered_.load()) return;

    const Eigen::Vector3d odom_p(odom_msg_.pose.pose.position.x,
                                 odom_msg_.pose.pose.position.y,
                                 odom_msg_.pose.pose.position.z);
    const Eigen::Vector3d odom_v(odom_msg_.twist.twist.linear.x,
                                 odom_msg_.twist.twist.linear.y,
                                 odom_msg_.twist.twist.linear.z);

    if (force_hover_ && odom_v.norm() > 0.1) return;

    Eigen::Vector3d goal(goal_x_, goal_y_, goal_z_);
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal - odom_p;
    if (delta.norm() < 15.0) {
      local_goal = goal;
    } else {
      local_goal = delta.normalized() * 15.0 + odom_p;
    }

    // obtain map
    map_ptr_->from_msg(map_msg_);

    // Determine whether to replan (port ROS1 fake_timer_callback behavior)
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_ && last_traj_.has_value()) {
      const double last_total = last_traj_->traj.getTotalDuration();
      const double last_traj_t_rest = last_total - (now() - replan_stamp_).seconds();
      const bool new_goal = (local_goal - last_traj_->traj.getPos(last_total)).norm() > tracking_dist_;
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          RCLCPP_WARN(get_logger(), "[planner(fake)] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(*last_traj_, replan_stamp_, last_traj_t_rest)) {
          RCLCPP_WARN(get_logger(), "[planner(fake)] NO NEED REPLAN...");
          const double t_delta = last_total < 1.0 ? last_total : 1.0;
          const double t_yaw = (now() - replan_stamp_).seconds() + t_delta;
          const Eigen::Vector3d un_known_p = last_traj_->traj.getPos(std::clamp(t_yaw, 0.0, last_total));
          const Eigen::Vector3d dp = un_known_p - odom_p;
          const double yaw = std::atan2(dp.y(), dp.x());
          // re-publish last traj to keep controller running (ROS1 behavior)
          PlannedTraj tr = *last_traj_;
          tr.yaw = static_cast<float>(yaw);
          publishTrajMsg(tr);
          no_need_replan = true;
        }
      }
    }

    // hover near goal
    if ((goal - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {
      if (!wait_hover_) {
        publishHoverAt(odom_p, now());
        wait_hover_ = true;
      }
      quadrotor_msgs::msg::ReplanState rs;
      rs.header.stamp = now();
      rs.header.frame_id = "world";
      rs.state = -1;
      rs.occmap = map_msg_;
      rs.replan_stamp = rs.header.stamp;
      replan_state_pub_->publish(rs);
      return;
    }
    wait_hover_ = false;

    if (no_need_replan) {
      return;
    }

    // init state
    const rclcpp::Time replan_stamp = now() + rclcpp::Duration::from_seconds(0.03);
    const double replan_t = (replan_stamp - replan_stamp_).seconds();
    Eigen::Vector3d ini_p, ini_v, ini_a;
    if (force_hover_ || !last_traj_.has_value() || replan_t > last_traj_->traj.getTotalDuration()) {
      ini_p = odom_p;
      ini_v = odom_v;
      ini_a.setZero();
    } else {
      ini_p = evalPos(*last_traj_, replan_t);
      ini_v = evalVel(*last_traj_, replan_t);
      ini_a = evalAcc(*last_traj_, replan_t);
    }

    quadrotor_msgs::msg::ReplanState rs;
    rs.header.stamp = now();
    rs.header.frame_id = "world";
    rs.occmap = map_msg_;
    rs.replan_stamp = replan_stamp;
    fillIniStateMsg(rs.ini_state, ini_p, ini_v, ini_a);

    std::vector<Eigen::Vector3d> path;
    bool ok = env_->astar_search(ini_p, local_goal, path);
    PlannedTraj new_traj;
    bool valid = false;
    if (ok && !path.empty()) {
      vis_->visualize_path(path, "astar");
      const Eigen::Vector3d fin_p = path.back();
      const double yaw = std::atan2((fin_p - ini_p).y(), (fin_p - ini_p).x());
      new_traj = makeQuinticTraj(ini_p, ini_v, ini_a, fin_p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), static_cast<float>(yaw), replan_stamp);
      valid = validcheck(new_traj, replan_stamp);
      if (!valid) {
        rs.state = -2;
        replan_state_pub_->publish(rs);
      }
    } else {
      rs.state = -2;
      replan_state_pub_->publish(rs);
    }

    if (valid) {
      force_hover_ = false;
      rs.state = 0;
      replan_state_pub_->publish(rs);
      publishTrajMsg(new_traj);
      last_traj_ = new_traj;
      replan_stamp_ = replan_stamp;
      return;
    }

    if (force_hover_) {
      rs.state = 1;
      replan_state_pub_->publish(rs);
      return;
    }
    if (last_traj_.has_value() && !validcheck(*last_traj_, replan_stamp_)) {
      force_hover_ = true;
      rs.state = 2;
      replan_state_pub_->publish(rs);
      publishHoverAt(ini_p, replan_stamp);
      return;
    }
    rs.state = 3;
    replan_state_pub_->publish(rs);
  }

  void debugTimerCb() {
    if (!debug_replan_state_.has_value()) return;
    inflate_gridmap_pub_->publish(debug_replan_state_->occmap);
    // Minimal ROS2 debug behavior: visualize ray + prediction + visible path from ini_state
    map_ptr_->from_msg(debug_replan_state_->occmap);
    predictor_->setMap(*map_ptr_);

    const auto& rs = *debug_replan_state_;
    if (rs.ini_state.size() < 6) return;
    Eigen::Vector3d ini_p(rs.ini_state[0], rs.ini_state[1], rs.ini_state[2]);
    Eigen::Vector3d ini_v(rs.ini_state.size() >= 6 ? rs.ini_state[3] : 0.0,
                          rs.ini_state.size() >= 6 ? rs.ini_state[4] : 0.0,
                          rs.ini_state.size() >= 6 ? rs.ini_state[5] : 0.0);
    Eigen::Vector3d target_p(rs.target.pose.pose.position.x,
                             rs.target.pose.pose.position.y,
                             rs.target.pose.pose.position.z);
    Eigen::Vector3d target_v(rs.target.twist.twist.linear.x,
                             rs.target.twist.twist.linear.y,
                             rs.target.twist.twist.linear.z);

    vis_->visualize_arrow(ini_p, ini_p + ini_v, "drone_vel", visualization_ros2::Color::greenblue);
    vis_->visualize_arrow(target_p, target_p + target_v, "target_vel", visualization_ros2::Color::greenblue);

    vis_->visualize_arrow(ini_p, target_p, "ray",
                          env_->checkRayValid(ini_p, target_p) ? visualization_ros2::Color::yellow
                                                               : visualization_ros2::Color::red);

    std::vector<Eigen::Vector3d> target_predict;
    if (!predictor_->predict(target_p, target_v, target_predict, 0.05)) return;
    vis_->visualize_path(target_predict, "car_predict");

    std::vector<Eigen::Vector3d> way_pts, path;
    if (env_->findVisiblePath(ini_p, target_predict, way_pts, path)) {
      vis_->visualize_path(path, "astar");
    }
  }

  // pubs/subs
  rclcpp::Publisher<quadrotor_msgs::msg::PolyTraj>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::ReplanState>::SharedPtr replan_state_pub_;
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr polyhedra_pub_;

  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;

  // params/state
  int plan_hz_{10};
  bool debug_{false};
  bool fake_{false};
  int16_t drone_id_{0};
  double tracking_dist_{2.0};
  double tolerance_d_{0.5};
  double tracking_dur_{1.0};
  double tracking_dt_{0.1};
  double rho_a_{1.0};
  double vmax_{5.0};
  double theta_clearance_{0.6};

  std::atomic_bool odom_received_{false};
  std::atomic_bool map_received_{false};
  std::atomic_bool target_received_{false};
  std::atomic_bool triggered_{false};
  std::atomic_bool land_triggered_{false};

  bool wait_hover_{true};
  bool force_hover_{true};
  rclcpp::Time replan_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<PlannedTraj> last_traj_;
  std::optional<quadrotor_msgs::msg::ReplanState> debug_replan_state_;

  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry target_msg_;
  quadrotor_msgs::msg::OccMap3d map_msg_;

  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_z_{0.9};
  int traj_id_{0};

  std::shared_ptr<visualization_ros2::Visualization> vis_;
  std::shared_ptr<prediction_ros2::Predict> predictor_;
  std::shared_ptr<mapping::OccGridMap> map_ptr_;
  std::shared_ptr<env_ros2::Env> env_;

  traj_opt::Params traj_opt_params_{};
  traj_opt::TrajOpt traj_opt_{};

  Eigen::Vector3d land_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond land_q_{Eigen::Quaterniond::Identity()};

  rclcpp::Publisher<quadrotor_msgs::msg::OccMap3d>::SharedPtr inflate_gridmap_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr land_trigger_sub_;
};

}  // namespace planning

RCLCPP_COMPONENTS_REGISTER_NODE(planning::PlanningComponent)

