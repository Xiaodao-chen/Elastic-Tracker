#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <traj_opt/poly_traj_utils.hpp>

class TrajServer : public rclcpp::Node {
public:
  TrajServer() : Node("traj_server") {
    poly_traj_sub_ = this->create_subscription<quadrotor_msgs::msg::PolyTraj>(
        "trajectory", 10,
        std::bind(&TrajServer::polyTrajCallback, this, std::placeholders::_1));
    
    heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "heartbeat", 10,
        std::bind(&TrajServer::heartbeatCallback, this, std::placeholders::_1));
    
    pos_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 50);
    
    cmd_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&TrajServer::cmdCallback, this));
    
    RCLCPP_WARN(this->get_logger(), "[Traj server]: ready.");
  }

private:
  void publish_cmd(int traj_id,
                   const Eigen::Vector3d &p,
                   const Eigen::Vector3d &v,
                   const Eigen::Vector3d &a,
                   double y, double yd) {
    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id;
    
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
  
  bool exe_traj(const quadrotor_msgs::msg::PolyTraj &trajMsg) {
    double t = (this->now() - trajMsg.start_time).seconds();
    if (t > 0) {
      if (trajMsg.hover) {
        if (trajMsg.hover_p.size() != 3) {
          RCLCPP_ERROR(this->get_logger(), "[traj_server] hover_p is not 3d!");
        }
        Eigen::Vector3d p, v0;
        p.x() = trajMsg.hover_p[0];
        p.y() = trajMsg.hover_p[1];
        p.z() = trajMsg.hover_p[2];
        v0.setZero();
        publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);
        return true;
      }
      if (trajMsg.order != 5) {
        RCLCPP_ERROR(this->get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
        return false;
      }
      if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
        RCLCPP_ERROR(this->get_logger(), "[traj_server] WRONG trajectory parameters!");
        return false;
      }
      int piece_nums = trajMsg.duration.size();
      std::vector<double> dura(piece_nums);
      std::vector<CoefficientMat> cMats(piece_nums);
      for (int i = 0; i < piece_nums; ++i) {
        int i6 = i * 6;
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
        RCLCPP_ERROR(this->get_logger(), "[traj_server] trajectory too short left!");
        return false;
      }
      Eigen::Vector3d p, v, a;
      p = traj.getPos(t);
      v = traj.getVel(t);
      a = traj.getAcc(t);
      // NOTE yaw
      double yaw = trajMsg.yaw;
      double d_yaw = yaw - last_yaw_;
      d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
      d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
      double d_yaw_abs = fabs(d_yaw);
      if (d_yaw_abs >= 0.02) {
        yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.02;
      }
      publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0);
      last_yaw_ = yaw;
      return true;
    }
    return false;
  }
  
  void heartbeatCallback(const std_msgs::msg::Empty::SharedPtr msg) {
    heartbeat_time_ = this->now();
  }
  
  void polyTrajCallback(const quadrotor_msgs::msg::PolyTraj::SharedPtr msgPtr) {
    trajMsg_ = *msgPtr;
    if (!receive_traj_) {
      trajMsg_last_ = trajMsg_;
      receive_traj_ = true;
    }
  }
  
  void cmdCallback() {
    if (!receive_traj_) {
      return;
    }
    rclcpp::Time time_now = this->now();
    if ((time_now - heartbeat_time_).seconds() > 0.5) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[traj_server] Lost heartbeat from the planner, is he dead?");
      publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0);
      return;
    }
    if (exe_traj(trajMsg_)) {
      trajMsg_last_ = trajMsg_;
      return;
    } else if (exe_traj(trajMsg_last_)) {
      return;
    }
  }
  
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Time heartbeat_time_;
  bool receive_traj_ = false;
  quadrotor_msgs::msg::PolyTraj trajMsg_, trajMsg_last_;
  Eigen::Vector3d last_p_;
  double last_yaw_ = 0;
  
  rclcpp::Subscription<quadrotor_msgs::msg::PolyTraj>::SharedPtr poly_traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajServer>());
  rclcpp::shutdown();
  return 0;
}
