#include "bound_visual.h"

namespace decomp_rviz_plugins {

BoundVisual::BoundVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
: frame_node_(parent_node->createChildSceneNode()), scene_manager_(scene_manager) {}

BoundVisual::~BoundVisual() {
  if (scene_manager_ && frame_node_) {
    scene_manager_->destroySceneNode(frame_node_);
  }
}

void BoundVisual::setMessage(const vec_E<vec_Vec3f>& bds) {
  objs_.clear();
  if (bds.empty()) {
    return;
  }

  objs_.resize(bds.size());
  for (auto &it : objs_) {
    it = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, frame_node_);
  }

  size_t cnt = 0;
  for (const auto &vs : bds) {
    for (unsigned int i = 0; i <= vs.size(); i++) {
      const auto& v = (i < vs.size()) ? vs[i] : vs[0];
      if (!std::isnan(v(0))) {
        objs_[cnt]->addPoint(Ogre::Vector3(v(0), v(1), v(2)));
      }
    }
    cnt++;
  }
}

void BoundVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void BoundVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void BoundVisual::setColor(float r, float g, float b, float a) {
  for (auto &it : objs_) {
    it->setColor(r, g, b, a);
  }
}

void BoundVisual::setScale(float s) {
  for (auto &it : objs_) {
    it->setLineWidth(s);
  }
}

}  // namespace decomp_rviz_plugins

