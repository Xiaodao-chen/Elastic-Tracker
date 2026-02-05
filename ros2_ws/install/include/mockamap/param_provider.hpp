#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace mockamap {

// Small helper to emulate ROS1 NodeHandle::param(name, var, default).
class ParamProvider {
 public:
  explicit ParamProvider(rclcpp::Node* node) : node_(node) {}

  template <typename T>
  void param(const std::string& name, T& out, const T& default_value) {
    if (!node_) {
      out = default_value;
      return;
    }
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    out = node_->get_parameter(name).get_value<T>();
  }

 private:
  rclcpp::Node* node_{nullptr};
};

}  // namespace mockamap

