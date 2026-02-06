#include <rclcpp/rclcpp.hpp>
#include "tracker_fsm/tracking_fsm.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tracker_node");
  
  tracker::TrackingFSM fsm;
  fsm.init(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
