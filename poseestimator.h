#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include "pcl_includes.h"
#include <pcl/filters/voxel_grid.h>

class PoseEstimator
{
public:
  PoseEstimator(float scene_leaf_size=0.005f, float object_leaf_size=0.005f);

  void set_scene_leaf_size(float s)  {scene_leaf_size = s;}
  void set_object_leaf_size(float s) {object_leaf_size = s;}
  float get_scene_leaf_size()  {return scene_leaf_size;}
  float get_object_leaf_size() {return object_leaf_size;}

  PointCloudT::Ptr const &process_scene (PointCloudT::Ptr const &in);
  PointCloudT::Ptr const &process_object(PointCloudT::Ptr const &in);

private:
  float scene_leaf_size, object_leaf_size;
  PointCloudT::Ptr scene_processed, object_processed;
  pcl::VoxelGrid<PointT> vox;
};

#endif // POSEESTIMATOR_H
