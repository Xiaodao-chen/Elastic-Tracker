#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_ERROR(rclcpp::get_logger("planning_node"),
               "Please run as a component. Example:\n"
               "  ros2 run rclcpp_components component_container_mt\n"
               "  ros2 component load /ComponentManager planning planning::PlanningComponent");
  rclcpp::shutdown();
  return 1;
}

