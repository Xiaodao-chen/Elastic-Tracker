#ifndef ELLIPSOIDS_VISUAL_H
#define ELLIPSOIDS_VISUAL_H

#include <decomp_basis/data_type.h>
#include <decomp_geometry/ellipsoid.h>

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>

#include <Eigen/Eigenvalues>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz_rendering/objects/shape.hpp>

namespace decomp_rviz_plugins {

class EllipsoidArrayVisual {
public:
  EllipsoidArrayVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  virtual ~EllipsoidArrayVisual();

  void setMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg);

  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setColor(float r, float g, float b, float a);

private:
  std::vector<std::unique_ptr<rviz_rendering::Shape>> objs_;
  Ogre::SceneNode *frame_node_{nullptr};
  Ogre::SceneManager *scene_manager_{nullptr};
};

}  // namespace decomp_rviz_plugins

#endif

