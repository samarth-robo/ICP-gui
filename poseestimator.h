#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include "pcl_includes.h"
#include <pcl/filters/voxel_grid.h>

class PoseEstimator
{
public:
  PoseEstimator(PointCloudT::ConstPtr const &scene = nullptr,
                PointCloudT::ConstPtr const &object = nullptr);

  // setters
  void set_scene_leaf_size(float s)  {scene_leaf_size = s;}
  void set_object_leaf_size(float s) {object_leaf_size = s;}
  void set_scene_min_height(float s) {scene_min_height = s;}
  void set_scene_boxsize_x(float s)  {scene_boxsize_x = s;}
  void set_scene_boxsize_y(float s)  {scene_boxsize_y = s;}
  void set_scene_boxsize_z(float s)  {scene_boxsize_z = s;}
  void set_object_init_x(float s)    {object_init_x = s;}
  void set_object_init_y(float s)    {object_init_y = s;}
  void set_object_init_z(float s)    {object_init_z = s;}
  void set_scene(PointCloudT::Ptr const &p);
  void set_object(PointCloudT::Ptr const &p);

  // getters
  float get_scene_leaf_size()  {return scene_leaf_size;}
  float get_object_leaf_size() {return object_leaf_size;}
  float get_scene_min_height() {return scene_min_height;}
  float get_scene_boxsize_x() {return scene_boxsize_x;}
  float get_scene_boxsize_y() {return scene_boxsize_y;}
  float get_scene_boxsize_z() {return scene_boxsize_z;}
  float get_object_init_x() {return object_init_x;}
  float get_object_init_y() {return object_init_y;}
  float get_object_init_z() {return object_init_z;}
  PointCloudT::Ptr const &get_processed_scene() {return scene_processed;}
  PointCloudT::Ptr const &get_processed_object() {return object_processed;}

  // other functions
  void process_scene();
  void process_object();

private:
  float scene_leaf_size, object_leaf_size;
  float scene_min_height;
  float scene_boxsize_x, scene_boxsize_y, scene_boxsize_z;
  float object_init_x, object_init_y, object_init_z;
  PointCloudT::Ptr scene_processed, object_processed;
  PointCloudT::ConstPtr scene, object;
  pcl::VoxelGrid<PointT> scene_vox, object_vox;
  pcl::ModelCoefficientsPtr scene_plane_coeffs;
  PointCloudT::Ptr scene_hull_points;

  // pose util functions
  typedef Eigen::Matrix4f tformT;
  tformT get_tabletop_rot(Eigen::Vector3f obj_normal = Eigen::Vector3f(0, 0, 1));
  tformT invert_pose(tformT const &in);
};

#endif // POSEESTIMATOR_H
