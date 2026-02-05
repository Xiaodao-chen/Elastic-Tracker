#include "polyhedron_array_display.h"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/logging.hpp>

namespace decomp_rviz_plugins {

PolyhedronArrayDisplay::PolyhedronArrayDisplay() {
  mesh_color_property_ = new rviz_common::properties::ColorProperty(
    "MeshColor", QColor(0, 170, 255), "Mesh color.", this, SLOT(updateMeshColorAndAlpha()));
  bound_color_property_ = new rviz_common::properties::ColorProperty(
    "BoundColor", QColor(255, 0, 0), "Bound color.", this, SLOT(updateBoundColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.2, "0 is fully transparent, 1.0 is fully opaque, only affect mesh", this,
    SLOT(updateMeshColorAndAlpha()));

  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 0.1, "bound scale.", this, SLOT(updateScale()));

  vs_scale_property_ = new rviz_common::properties::FloatProperty(
    "VsScale", 1.0, "Vs scale.", this, SLOT(updateVsScale()));

  vs_color_property_ = new rviz_common::properties::ColorProperty(
    "VsColor", QColor(0, 255, 0), "Vs color.", this, SLOT(updateVsColorAndAlpha()));

  state_property_ = new rviz_common::properties::EnumProperty(
    "State", "Mesh",
    "A Polygon can be represented as two states: Mesh and Bound, this option allows selecting "
    "visualizing Polygon in corresponding state",
    this, SLOT(updateState()));
  state_property_->addOption("Mesh", 0);
  state_property_->addOption("Bound", 1);
  state_property_->addOption("Both", 2);
  state_property_->addOption("Vs", 3);
}

void PolyhedronArrayDisplay::onInitialize() {
  MFDClass::onInitialize();
}

void PolyhedronArrayDisplay::reset() {
  MFDClass::reset();
  visual_mesh_.reset();
  visual_bound_.reset();
  visual_vector_.reset();
}

void PolyhedronArrayDisplay::processMessage(decomp_ros_msgs::msg::PolyhedronArray::ConstSharedPtr msg) {
  if (!context_->getFrameManager()->getTransform(msg->header, position_, orientation_)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "Error transforming from frame '" << msg->header.frame_id << "' to fixed frame");
    return;
  }

  vertices_.clear();
  vs_.clear();

  const auto polys = DecompROS::ros_to_polyhedron_array(*msg);
  for (const auto& polyhedron: polys) {
    vec_E<vec_Vec3f> bds = cal_vertices(polyhedron);
    vertices_.insert(vertices_.end(), bds.begin(), bds.end());
    const auto vs = polyhedron.cal_normals();
    vs_.insert(vs_.end(), vs.begin(), vs.end());
  }

  const int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedronArrayDisplay::visualizeMesh() {
  auto visual_mesh = std::make_shared<MeshVisual>(context_->getSceneManager(), scene_node_);

  visual_mesh->setMessage(vertices_);
  visual_mesh->setFramePosition(position_);
  visual_mesh->setFrameOrientation(orientation_);

  const float alpha = alpha_property_->getFloat();
  const Ogre::ColourValue color = mesh_color_property_->getOgreColor();
  visual_mesh->setColor(color.r, color.g, color.b, alpha);
  visual_mesh_ = visual_mesh;
}

void PolyhedronArrayDisplay::visualizeBound() {
  auto visual_bound = std::make_shared<BoundVisual>(context_->getSceneManager(), scene_node_);

  visual_bound->setMessage(vertices_);
  visual_bound->setFramePosition(position_);
  visual_bound->setFrameOrientation(orientation_);

  const Ogre::ColourValue color = bound_color_property_->getOgreColor();
  visual_bound->setColor(color.r, color.g, color.b, 1.0);
  const float scale = scale_property_->getFloat();
  visual_bound->setScale(scale);

  visual_bound_ = visual_bound;
}

void PolyhedronArrayDisplay::visualizeVs() {
  auto visual_vector = std::make_shared<VectorVisual>(context_->getSceneManager(), scene_node_);

  visual_vector->setMessage(vs_);
  visual_vector->setFramePosition(position_);
  visual_vector->setFrameOrientation(orientation_);

  const Ogre::ColourValue color = vs_color_property_->getOgreColor();
  visual_vector->setColor(color.r, color.g, color.b, 1.0);

  const float s = vs_scale_property_->getFloat();
  visual_vector->setScale(s);
  visual_vector_ = visual_vector;
}

void PolyhedronArrayDisplay::visualizeMessage(int state) {
  switch (state) {
    case 0:
      visual_bound_.reset();
      visual_vector_.reset();
      visualizeMesh();
      break;
    case 1:
      visual_mesh_.reset();
      visual_vector_.reset();
      visualizeBound();
      break;
    case 2:
      visual_vector_.reset();
      visualizeMesh();
      visualizeBound();
      break;
    case 3:
      visual_mesh_.reset();
      visual_bound_.reset();
      visualizeVs();
      break;
    default:
      RVIZ_COMMON_LOG_WARNING_STREAM("Invalid State: " << state);
  }
}

void PolyhedronArrayDisplay::updateMeshColorAndAlpha() {
  const float alpha = alpha_property_->getFloat();
  const Ogre::ColourValue color = mesh_color_property_->getOgreColor();
  if (visual_mesh_) {
    visual_mesh_->setColor(color.r, color.g, color.b, alpha);
  }
}

void PolyhedronArrayDisplay::updateBoundColorAndAlpha() {
  const Ogre::ColourValue color = bound_color_property_->getOgreColor();
  if (visual_bound_) {
    visual_bound_->setColor(color.r, color.g, color.b, 1.0);
  }
}

void PolyhedronArrayDisplay::updateState() {
  const int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedronArrayDisplay::updateScale() {
  const float s = scale_property_->getFloat();
  if (visual_bound_) {
    visual_bound_->setScale(s);
  }
}

void PolyhedronArrayDisplay::updateVsScale() {
  const float s = vs_scale_property_->getFloat();
  if (visual_vector_) {
    visual_vector_->setScale(s);
  }
}

void PolyhedronArrayDisplay::updateVsColorAndAlpha() {
  const Ogre::ColourValue color = vs_color_property_->getOgreColor();
  if (visual_vector_) {
    visual_vector_->setColor(color.r, color.g, color.b, 1.0);
  }
}

}  // namespace decomp_rviz_plugins

PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::PolyhedronArrayDisplay, rviz_common::Display)

