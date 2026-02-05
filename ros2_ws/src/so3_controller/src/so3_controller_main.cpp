#include <rclcpp/rclcpp.hpp>

#include <so3_controller/so3_controller.hpp>

// The component class lives in the shared library; include its header via forward declaration
// by re-including the translation unit symbol is not possible, so we just create it through
// rclcpp_components at runtime in typical usage. For convenience, we provide a simple
// standalone that loads the component library automatically is overkill here; instead we
// compile the component into the executable by linking the shared lib and referencing the type.

namespace so3_controller {
class SO3ControllerComponent;
}  // namespace so3_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Note: We cannot instantiate the component type without its full definition here.
  // Users should run it via component container:
  //   ros2 run rclcpp_components component_container_mt
  //   ros2 component load /ComponentManager so3_controller so3_controller::SO3ControllerComponent
  RCLCPP_ERROR(rclcpp::get_logger("so3_controller_node"),
               "Please run as a component. Example:\n"
               "  ros2 run rclcpp_components component_container_mt\n"
               "  ros2 component load /ComponentManager so3_controller so3_controller::SO3ControllerComponent");
  rclcpp::shutdown();
  return 1;
}

