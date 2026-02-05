#ifndef DECOMP_RVIZ_PLUGINS__ELLIPSOID_ARRAY_DISPLAY_HPP_
#define DECOMP_RVIZ_PLUGINS__ELLIPSOID_ARRAY_DISPLAY_HPP_

#include <memory>

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include "ellipsoid_array_visual.h"

namespace decomp_rviz_plugins {

class EllipsoidArrayDisplay
  : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::EllipsoidArray>
{
  Q_OBJECT

public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay() override = default;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateColorAndAlpha();

private:
  void processMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg) override;

  std::shared_ptr<EllipsoidArrayVisual> visual_;
  rviz_common::properties::ColorProperty *color_property_{nullptr};
  rviz_common::properties::FloatProperty *alpha_property_{nullptr};
};

}  // namespace decomp_rviz_plugins

#endif

