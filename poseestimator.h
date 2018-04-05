#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include "pcl_includes.h"
#include <pcl/filters/voxel_grid.h>

class PoseEstimator
{
private:
  typedef Eigen::Matrix4f tformT;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  void set_object_init_azim(float s) {object_init_azim = s;}
  void set_icp_outlier_dist(float s) {icp_outlier_rejection_thresh = s;}
  void set_icp_corr_dist(float s)    {icp_max_corr_distance = s;}
  void set_icp_use_recip_corr(bool s){icp_use_reciprocal_corr = s;}
  void set_icp_estimate_scale(bool s){icp_estimate_scale = s;}
  void set_scale_axis(char a)        {scale_axis = a;}
  void set_scene(PointCloudT::Ptr const &p);
  void set_object(PointCloudT::Ptr const &p);

  // getters
  float get_scene_leaf_size()    {return scene_leaf_size;}
  float get_object_leaf_size()   {return object_leaf_size;}
  float get_scene_min_height()   {return scene_min_height;}
  float get_scene_boxsize_x()    {return scene_boxsize_x;}
  float get_scene_boxsize_y()    {return scene_boxsize_y;}
  float get_scene_boxsize_z()    {return scene_boxsize_z;}
  pcl::PointXYZ get_scene_box_min_pt();
  pcl::PointXYZ get_scene_box_max_pt();
  float get_object_init_x()      {return object_init_x;}
  float get_object_init_y()      {return object_init_y;}
  float get_object_init_z()      {return object_init_z;}
  float get_object_init_azim()   {return object_init_azim;}
  float get_icp_outlier_dist()   {return icp_outlier_rejection_thresh;}
  float get_icp_corr_dist()      {return icp_max_corr_distance;}
  bool  get_icp_use_recip_corr() {return icp_use_reciprocal_corr;}
  bool  get_icp_estimate_scale() {return icp_estimate_scale;}
  char  get_scale_axis()         {return scale_axis;}
  PointCloudT::ConstPtr get_processed_scene() {return scene_processed;}
  PointCloudT::ConstPtr get_processed_object();
  PointCloudT::ConstPtr get_scene() {return scene;}
  PointCloudT::ConstPtr get_cropped_subsampled_scene()
  {return scene_cropped_subsampled;}
  pcl::ModelCoefficientsConstPtr get_scene_plane_coeffs()
  {return scene_plane_coeffs;}
  tformT get_object_pose() {return object_pose;}

  // other functions
  void process_scene();
  void process_object();
  void init_icp();
  bool do_icp();
  bool write_pose_file(std::string pose_filename, std::string scale_filename);
  bool estimate_plane_params();

private:
  void crop_subsample_scene();
  // pose util functions
  tformT get_tabletop_rot(Eigen::Vector3f obj_normal = Eigen::Vector3f(0, 0, 1));
  tformT invert_pose(tformT const &in);

  float scene_leaf_size, object_leaf_size;
  float scene_min_height;
  float scene_boxsize_x, scene_boxsize_y, scene_boxsize_z;
  float object_init_x, object_init_y, object_init_z, object_init_azim;
  float icp_outlier_rejection_thresh, icp_max_corr_distance;
  size_t icp_n_iters;
  bool icp_use_reciprocal_corr, icp_estimate_scale;
  PointCloudT::ConstPtr scene, object;
  PointCloudT::Ptr scene_cropped_subsampled;
  PointCloudT::Ptr scene_processed, object_processed;
  pcl::VoxelGrid<PointT> scene_vox, object_vox;
  pcl::ModelCoefficientsPtr scene_plane_coeffs;
  PointCloudT::Ptr scene_plane_hull_points;
  tformT object_pose, object_azim, object_scale;
  float axis_size;  // size of object along an axis in the scene
  char scale_axis;  // which axis to use for scaling
};

#endif // POSEESTIMATOR_H
