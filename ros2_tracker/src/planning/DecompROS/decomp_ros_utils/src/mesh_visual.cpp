#include "mesh_visual.h"

namespace decomp_rviz_plugins {

int MeshVisual::count_ = 0;

MeshVisual::MeshVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  material_name_ = "DecompMeshMaterial_" + std::to_string(count_++);
  material_ = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
  material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

  manual_object_ = scene_manager_->createManualObject();
  frame_node_->attachObject(manual_object_);
}

MeshVisual::~MeshVisual() {
  scene_manager_->destroyManualObject(manual_object_);
  Ogre::MaterialManager::getSingleton().remove(material_name_);
  scene_manager_->destroySceneNode(frame_node_);
}

void MeshVisual::setMessage(const vec_E<vec_Vec3f> &bds) {
  manual_object_->clear();

  if (bds.empty())
    return;

  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  int free_cnt = 0;
  for (const auto &vs : bds) {
    if (vs.size() > 2) {
      Vec3f p0 = vs[0];
      Vec3f p1 = vs[1];
      Vec3f p2 = vs[2];
      Vec3f n = (p2 - p0).cross(p1 - p0);
      n = n.normalized();
      if (std::isnan(n(0)))
        n = Vec3f(0, 0, -1);

      int ref_cnt = free_cnt;
      Ogre::Vector3 normal(n(0), n(1), n(2));
      for (unsigned int i = 0; i < vs.size(); i++) {
        manual_object_->position(Ogre::Vector3(vs[i](0), vs[i](1), vs[i](2)));
        manual_object_->normal(normal);
        if (i > 1 && i < vs.size())
          manual_object_->triangle(ref_cnt, free_cnt - 1, free_cnt);
        free_cnt++;
      }
    }
  }
  manual_object_->end();
}

void MeshVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void MeshVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void MeshVisual::setColor(float r, float g, float b, float a) {
  material_->getTechnique(0)->getPass(0)->setDiffuse(r, g, b, a);
  material_->getTechnique(0)->getPass(0)->setAmbient(r, g, b);
}

}
