#include "poseestimator.h"


using namespace pcl;

PoseEstimator::PoseEstimator(float scene_leaf_size, float object_leaf_size,
                             PointCloudT::ConstPtr const &scene_,
                             PointCloudT::ConstPtr const &object_) :
  scene_leaf_size(scene_leaf_size), object_leaf_size(object_leaf_size),
  scene_processed(new PointCloudT()), object_processed(new PointCloudT) {
  if (scene_) {
    scene = scene_;
    scene_vox.setInputCloud(scene);
  }
  if (object_) {
    object = object_;
    object_vox.setInputCloud(object);
  }
}

PointCloudT::Ptr const &PoseEstimator::process_scene() {
  scene_vox.setLeafSize(scene_leaf_size, scene_leaf_size, scene_leaf_size);
  scene_vox.filter(*scene_processed);
  return scene_processed;
}

PointCloudT::Ptr const &PoseEstimator::process_object() {
  object_vox.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  object_vox.filter(*object_processed);
  return object_processed;
}

void PoseEstimator::set_scene(const PointCloudT::Ptr &p) {
  scene = p;
  scene_vox.setInputCloud(scene);
}

void PoseEstimator::set_object(const PointCloudT::Ptr &p) {
  object = p;
  object_vox.setInputCloud(object);
}
