#include <rclcpp/rclcpp.hpp>
#include "tracker_fsm/tracking_fsm.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tracker_node");
  
  tracker::TrackingFSM fsm;
  fsm.init(node);
  
  // Use a multi-threaded executor so timer-based planning doesn't block
  // high-rate subscriptions (odom/map/target). This reduces jitter and CPU
  // wasted on busy waiting.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2 /* threads */);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
