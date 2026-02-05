#include <rclcpp/rclcpp.hpp>

#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>

#include <traj_opt/poly_traj_utils.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>

namespace planning_ros2 {

class TrajServerNode : public rclcpp::Node {
 public:
  TrajServerNode() : rclcpp::Node("traj_server") {
    poly_traj_sub_ = create_subscription<quadrotor_msgs::msg::PolyTraj>(
      "trajectory", 10, std::bind(&TrajServerNode::polyTrajCallback, this, std::placeholders::_1));
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      "heartbeat", 10, std::bind(&TrajServerNode::heartbeatCallback, this, std::placeholders::_1));

    pos_cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 50);

    heartbeat_time_ = now();
    last_p_.setZero();

    using namespace std::chrono_literals;
    cmd_timer_ = create_wall_timer(10ms, std::bind(&TrajServerNode::cmdCallback, this));

    RCLCPP_WARN(get_logger(), "[Traj server]: ready.");
  }

 private:
  void publishCmd(int traj_id,
                  const Eigen::Vector3d& p,
                  const Eigen::Vector3d& v,
                  const Eigen::Vector3d& a,
                  double y,
                  double yd) {
    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = static_cast<uint32_t>(traj_id);

    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    cmd.yaw = y;
    cmd.yaw_dot = yd;

    pos_cmd_pub_->publish(cmd);
    last_p_ = p;
  }

  bool exeTraj(const quadrotor_msgs::msg::PolyTraj& trajMsg) {
    const double t = (now() - rclcpp::Time(trajMsg.start_time)).seconds();
    if (t <= 0) {
      return false;
    }

    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        RCLCPP_ERROR(get_logger(), "[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p(trajMsg.hover_p.size() > 0 ? trajMsg.hover_p[0] : 0.0,
                        trajMsg.hover_p.size() > 1 ? trajMsg.hover_p[1] : 0.0,
                        trajMsg.hover_p.size() > 2 ? trajMsg.hover_p[2] : 0.0);
      Eigen::Vector3d z = Eigen::Vector3d::Zero();
      publishCmd(trajMsg.traj_id, p, z, z, last_yaw_, 0.0);
      return true;
    }

    if (trajMsg.order != 5) {
      RCLCPP_ERROR(get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * static_cast<size_t>(trajMsg.order + 1) != trajMsg.coef_x.size()) {
      RCLCPP_ERROR(get_logger(), "[traj_server] WRONG trajectory parameters!");
      return false;
    }

    const int piece_nums = static_cast<int>(trajMsg.duration.size());
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);

    for (int i = 0; i < piece_nums; ++i) {
      const int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
        trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
        trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
        trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];
      dura[i] = trajMsg.duration[i];
    }

    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      RCLCPP_ERROR(get_logger(), "[traj_server] trajectory too short left!");
      return false;
    }

    const Eigen::Vector3d p = traj.getPos(t);
    const Eigen::Vector3d v = traj.getVel(t);
    const Eigen::Vector3d a = traj.getAcc(t);

    // yaw slew rate limiting (copy ROS1 behavior)
    double yaw = trajMsg.yaw;
    double d_yaw = yaw - last_yaw_;
    d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
    d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
    const double d_yaw_abs = fabs(d_yaw);
    if (d_yaw_abs >= 0.02) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.02;
    }

    publishCmd(trajMsg.traj_id, p, v, a, yaw, 0.0);
    last_yaw_ = yaw;
    return true;
  }

  void heartbeatCallback(const std_msgs::msg::Empty::ConstSharedPtr) {
    heartbeat_time_ = now();
  }

  void polyTrajCallback(const quadrotor_msgs::msg::PolyTraj::ConstSharedPtr msg) {
    trajMsg_ = *msg;
    if (!receive_traj_) {
      trajMsg_last_ = trajMsg_;
      receive_traj_ = true;
    }
  }

  void cmdCallback() {
    if (!receive_traj_) {
      return;
    }

    const auto time_now = now();
    if ((time_now - heartbeat_time_).seconds() > 0.5) {
      RCLCPP_ERROR_ONCE(get_logger(), "[traj_server] Lost heartbeat from the planner, is he dead?");
      publishCmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0);
      return;
    }

    if (exeTraj(trajMsg_)) {
      trajMsg_last_ = trajMsg_;
      return;
    }
    (void)exeTraj(trajMsg_last_);
  }

  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PolyTraj>::SharedPtr poly_traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  rclcpp::Time heartbeat_time_{0, 0, RCL_ROS_TIME};
  bool receive_traj_{false};
  quadrotor_msgs::msg::PolyTraj trajMsg_{};
  quadrotor_msgs::msg::PolyTraj trajMsg_last_{};

  Eigen::Vector3d last_p_{Eigen::Vector3d::Zero()};
  double last_yaw_{0.0};
};

}  // namespace planning_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planning_ros2::TrajServerNode>());
  rclcpp::shutdown();
  return 0;
}

