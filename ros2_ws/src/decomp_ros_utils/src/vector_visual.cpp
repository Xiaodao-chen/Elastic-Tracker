#include "vector_visual.h"

namespace decomp_rviz_plugins {

VectorVisual::VectorVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
: frame_node_(parent_node->createChildSceneNode()), scene_manager_(scene_manager) {}

VectorVisual::~VectorVisual() {
  if (scene_manager_ && frame_node_) {
    scene_manager_->destroySceneNode(frame_node_);
  }
}

void VectorVisual::setMessage(const vec_E<std::pair<Vec3f, Vec3f>> &vs) {
  objs_.clear();
  vs_ = vs;
  if (vs.empty()) {
    return;
  }

  objs_.resize(vs.size());
  for (auto &it : objs_) {
    it = std::make_unique<rviz_rendering::Arrow>(scene_manager_, frame_node_);
  }

  size_t cnt = 0;
  for (const auto &v : vs) {
    Vec3f n = v.second.normalized();
    objs_[cnt]->setDirection(Ogre::Vector3(n(0), n(1), n(2)));
    objs_[cnt]->setPosition(Ogre::Vector3(v.first(0), v.first(1), v.first(2)));
    objs_[cnt]->setScale(s_ * Ogre::Vector3(1.0, 1.0, 1.0));

    float d = s_ * v.second.norm();
    float shaft_length = 0.7f * d;
    float head_length = 0.3f * d;
    objs_[cnt]->set(shaft_length, d / 8.0f, head_length, d / 5.0f);
    cnt++;
  }
}

void VectorVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void VectorVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void VectorVisual::setColor(float r, float g, float b, float a) {
  for (auto &it : objs_) {
    it->setColor(r, g, b, a);
  }
}

void VectorVisual::setScale(float s) {
  s_ = s;
  for (size_t i = 0; i < objs_.size(); i++) {
    float d = s_ * vs_[i].second.norm();
    float shaft_length = 0.7f * d;
    float head_length = 0.3f * d;
    objs_[i]->set(shaft_length, d / 8.0f, head_length, d / 5.0f);
  }
}

}  // namespace decomp_rviz_plugins

