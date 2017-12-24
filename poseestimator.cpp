#include "poseestimator.h"


using namespace pcl;

PoseEstimator::PoseEstimator(float scene_leaf_size, float object_leaf_size) :
  scene_leaf_size(scene_leaf_size), object_leaf_size(object_leaf_size),
  scene_processed(new PointCloudT()), object_processed(new PointCloudT) {}

PointCloudT::Ptr const &PoseEstimator::process_scene(PointCloudT::Ptr const &in) {
  vox.setLeafSize(scene_leaf_size, scene_leaf_size, scene_leaf_size);
  vox.setInputCloud(in);
  vox.filter(*scene_processed);
  return scene_processed;
}

PointCloudT::Ptr const &PoseEstimator::process_object(PointCloudT::Ptr const &in) {
  vox.setLeafSize(scene_leaf_size, scene_leaf_size, scene_leaf_size);
  vox.setInputCloud(in);
  vox.filter(*object_processed);
  return object_processed;
}
