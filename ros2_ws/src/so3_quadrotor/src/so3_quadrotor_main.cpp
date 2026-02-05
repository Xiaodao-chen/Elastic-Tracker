#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_ERROR(rclcpp::get_logger("so3_quadrotor_node"),
               "Please run as a component. Example:\n"
               "  ros2 run rclcpp_components component_container_mt\n"
               "  ros2 component load /ComponentManager so3_quadrotor so3_quadrotor::So3QuadrotorComponent");
  rclcpp::shutdown();
  return 1;
}

