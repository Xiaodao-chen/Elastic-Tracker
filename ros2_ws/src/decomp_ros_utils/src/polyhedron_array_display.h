#ifndef DECOMP_RVIZ_PLUGINS__POLYHEDRON_ARRAY_DISPLAY_HPP_
#define DECOMP_RVIZ_PLUGINS__POLYHEDRON_ARRAY_DISPLAY_HPP_

#include <memory>

#include <decomp_ros_msgs/msg/polyhedron_array.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "mesh_visual.h"
#include "bound_visual.h"
#include "vector_visual.h"

#include <decomp_geometry/geometric_utils.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace decomp_rviz_plugins {

class PolyhedronArrayDisplay
  : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::PolyhedronArray>
{
  Q_OBJECT

public:
  PolyhedronArrayDisplay();
  ~PolyhedronArrayDisplay() override = default;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateMeshColorAndAlpha();
  void updateBoundColorAndAlpha();
  void updateVsColorAndAlpha();
  void updateState();
  void updateScale();
  void updateVsScale();

private:
  void processMessage(decomp_ros_msgs::msg::PolyhedronArray::ConstSharedPtr msg) override;
  void visualizeMessage(int state);
  void visualizeMesh();
  void visualizeBound();
  void visualizeVs();

  std::shared_ptr<MeshVisual> visual_mesh_;
  std::shared_ptr<BoundVisual> visual_bound_;
  std::shared_ptr<VectorVisual> visual_vector_;

  rviz_common::properties::ColorProperty *mesh_color_property_{nullptr};
  rviz_common::properties::ColorProperty *bound_color_property_{nullptr};
  rviz_common::properties::ColorProperty *vs_color_property_{nullptr};
  rviz_common::properties::FloatProperty *alpha_property_{nullptr};
  rviz_common::properties::FloatProperty *scale_property_{nullptr};
  rviz_common::properties::FloatProperty *vs_scale_property_{nullptr};
  rviz_common::properties::EnumProperty *state_property_{nullptr};

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  vec_E<vec_Vec3f> vertices_;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};

}  // namespace decomp_rviz_plugins

#endif

