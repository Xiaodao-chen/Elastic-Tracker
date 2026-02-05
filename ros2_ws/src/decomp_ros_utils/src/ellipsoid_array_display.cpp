#include "ellipsoid_array_display.h"

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <pluginlib/class_list_macros.hpp>

#include <rviz_common/logging.hpp>

namespace decomp_rviz_plugins {

EllipsoidArrayDisplay::EllipsoidArrayDisplay() {
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(204, 51, 204), "Color of ellipsoids.", this, SLOT(updateColorAndAlpha()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));
}

void EllipsoidArrayDisplay::onInitialize() {
  MFDClass::onInitialize();
}

void EllipsoidArrayDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void EllipsoidArrayDisplay::updateColorAndAlpha() {
  const float alpha = alpha_property_->getFloat();
  const Ogre::ColourValue color = color_property_->getOgreColor();
  if (visual_) {
    visual_->setColor(color.r, color.g, color.b, alpha);
  }
}

void EllipsoidArrayDisplay::processMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "Error transforming from frame '" << msg->header.frame_id << "' to fixed frame");
    return;
  }

  auto visual = std::make_shared<EllipsoidArrayVisual>(context_->getSceneManager(), scene_node_);
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  const float alpha = alpha_property_->getFloat();
  const Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
}

}  // namespace decomp_rviz_plugins

PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::EllipsoidArrayDisplay, rviz_common::Display)

