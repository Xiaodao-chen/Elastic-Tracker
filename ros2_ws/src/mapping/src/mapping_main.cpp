#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_ERROR(rclcpp::get_logger("mapping_node"),
               "Please run as a component. Example:\n"
               "  ros2 run rclcpp_components component_container_mt\n"
               "  ros2 component load /ComponentManager mapping mapping::MappingComponent");
  rclcpp::shutdown();
  return 1;
}

