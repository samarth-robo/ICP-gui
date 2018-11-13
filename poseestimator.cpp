#include "poseestimator.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/registration.h>

#include <fstream>
#include <math.h>

using namespace pcl;

PoseEstimator::PoseEstimator(PointCloudT::ConstPtr const &scene_,
                             PointCloudT::ConstPtr const &object_) :
  scene_leaf_size(0.001f), object_leaf_size(0.001f),
  scene_boxsize_x(0.3f), scene_boxsize_y(0.3f), scene_boxsize_z(0.3f),
  scene_distorted(false),
  object_init_x(0.f), object_init_y(0.f), object_init_z(0.f),
  object_init_azim(0.f),
  icp_n_iters(200), icp_outlier_rejection_thresh(0.005),
  icp_max_corr_distance(0.02), icp_use_reciprocal_corr(false),
  icp_no_rollpitch(false), icp_symmetric_object(false), icp_only_xy(false),
  scale_axis('z'),
  object_adj_pos(PoseEstimator::tformT::Identity()),
  object_adj_rot(PoseEstimator::tformT::Identity()),
  height_adjust(0.000),
  object_scale(PoseEstimator::tformT::Identity()),
  object_init_dx(0), object_init_dy(0), object_init_dz(0),
  forced_object_scale(1.f),
  T_f_o(PoseEstimator::tformT::Identity()),
  T_icp(PoseEstimator::tformT::Identity()),
  object_flip(PoseEstimator::tformT::Identity()),
  white_thresh(-1),
  te_lm(boost::make_shared<TELM>()),
  warp_no_azim(boost::make_shared<WarpNoAzim>()),
  warp_no_rotation(boost::make_shared<WarpNoRotation>()),
  warp_no_rollpitch(boost::make_shared<WarpNoRollPitch>()),
  warp_xy(boost::make_shared<WarpXY>()),
  warp_6d(boost::make_shared<Warp6D>()),
  T_b_f_offset(PoseEstimator::tformT::Identity()),
  T_b_f_offset_locked(false),
  azim_search_range(360.f),
  azim_search_step(45.f) {
  if (scene_) {
    scene = scene_;
    scene_vox.setInputCloud(scene);
  }
  if (object_) {
    object = object_;
    object_vox.setInputCloud(object);
  }

  // init the various shared pointers
  scene_cropped_subsampled = boost::make_shared<PointCloudT>();
  scene_processed          = boost::make_shared<PointCloudT>();
  object_processed         = boost::make_shared<PointCloudT>();
  scene_plane_hull_points  = boost::make_shared<PointCloudT>();
  scene_object_segmented   = boost::make_shared<PointCloudT>();
  scene_plane_coeffs       = boost::make_shared<ModelCoefficients>();
  distorted_object_plane_coeffs = boost::make_shared<ModelCoefficients>();
}

void PoseEstimator::set_scene(const PointCloudT::Ptr &p) {
  scene = p;
  scene_vox.setInputCloud(scene);
  // crop and subsample scene
  crop_subsample_scene();
  PointT min_pt, max_pt;
  getMinMax3D<PointT>(*scene, min_pt, max_pt);
}

void PoseEstimator::set_object(const PointCloudT::Ptr &p) {
  object = p;
  object_vox.setInputCloud(object);
  object_adj_pos = tformT::Identity();
  object_adj_rot = tformT::Identity();
  object_scale = tformT::Identity();
  PointT min_pt, max_pt;
  getMinMax3D<PointT>(*object, min_pt, max_pt);
}

void PoseEstimator::set_tt_pose(const tformT &T) {
  set_object_init_x(T(0, 3));
  set_object_init_y(T(1, 3));
  set_object_init_z(T(2, 3));
  Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  tt_axis = R * Eigen::Vector3f(0, 0, 1);
}

PointXYZ PoseEstimator::get_scene_box_min_pt() {
    PointXYZ p;
    p.x = object_init_x - scene_boxsize_x/2;
    // p.y = object_init_y - scene_boxsize_y;
    p.y = object_init_y - scene_boxsize_y + 0.05;
    p.z = object_init_z - scene_boxsize_z/2;
    return p;
}

PointXYZ PoseEstimator::get_scene_box_max_pt() {
    PointXYZ p;
    p.x = object_init_x + scene_boxsize_x/2;
    // p.y = object_init_y + scene_boxsize_y/2;
    p.y = object_init_y + 0.05;
    p.z = object_init_z + scene_boxsize_z/2;
    return p;
}

void PoseEstimator::crop_subsample_scene() {
  // subsample using voxel grid
  scene_vox.setLeafSize(scene_leaf_size, scene_leaf_size, scene_leaf_size);
  PointCloudT::Ptr subs = boost::make_shared<PointCloudT>();
  scene_vox.filter(*subs);

  CropBox<PointT> box;
  box.setInputCloud(subs);
  box.setMin(Eigen::Vector4f(-scene_boxsize_x/2, -scene_boxsize_y+0.05,
                             -scene_boxsize_z/2, 0));
  box.setMax(Eigen::Vector4f(+scene_boxsize_x/2, 0.05, +scene_boxsize_z/2, 0));
  box.setTranslation(Eigen::Vector3f(object_init_x, object_init_y,
                                     object_init_z));
  box.filter(*scene_cropped_subsampled);
}

bool PoseEstimator::set_T_b_f(std::string filename) {
  ifstream f(filename);
  if (!f.is_open()) {
    cerr << "Could not open " << filename << " for reading" << endl;
    return false;
  }

  T_b_f = tformT::Identity();
  f >> T_b_f(0, 3);
  f >> T_b_f(1, 3);
  f >> T_b_f(2, 3);
  f >> T_b_f(0, 0);
  f >> T_b_f(0, 1);
  f >> T_b_f(0, 2);
  f >> T_b_f(1, 0);
  f >> T_b_f(1, 1);
  f >> T_b_f(1, 2);
  f >> T_b_f(2, 0);
  f >> T_b_f(2, 1);
  f >> T_b_f(2, 2);
  f.close();

  if (!T_b_f_offset_locked) {
    T_b_f_offset = invert_pose(T_b_f);
    T_b_f_offset_locked = true;
  }
  return true;
}


bool PoseEstimator::estimate_plane_params(std::string from_filename) {
  // estimate the plane
  // console::print_info("Estimating plane...");
  if (!from_filename.empty()) {  // take plane estimate from the file
    ifstream f(from_filename);
    if (f.is_open()) {
      std::string line;
      getline(f, line);
      getline(f, line);
      std::stringstream ss(line);
      scene_plane_coeffs->values.resize(4);
      ss >> scene_plane_coeffs->values[0];
      ss >> scene_plane_coeffs->values[1];
      ss >> scene_plane_coeffs->values[2];
      ss >> scene_plane_coeffs->values[3];
    } else {
      console::print_error("Could not open %s for reading, estimating plane",
                           from_filename.c_str());
      return false;
    }
  } else {  // estimate plane by RANSAC
    PointCloudT::Ptr tt_points = boost::make_shared<PointCloudT>();
    if (white_thresh > 0) {
      // keep only non-white points for better plane estimate
      PointIndicesPtr idx = boost::make_shared<PointIndices>();
      for (int i=0; i<scene_cropped_subsampled->size(); i++) {
        const auto &p = scene_cropped_subsampled->at(i);
        float gray = 0.21*p.r + 0.72*p.g + 0.07*p.b;
        if (gray < white_thresh) idx->indices.push_back(i);
      }
      ExtractIndices<PointT> extract;
      extract.setInputCloud(scene_cropped_subsampled);
      extract.setNegative(false);
      extract.setIndices(idx);
      extract.filter(*tt_points);
    } else copyPointCloud(*scene_cropped_subsampled, *tt_points);

    if (!estimate_perpendicular_plane(tt_points, tt_axis, scene_plane_coeffs,
                                      nullptr, nullptr, 10*M_PI/180.f)) {
      return false;
    }
  }

  // flip the plane normal if needed
  Eigen::Vector3f n(scene_plane_coeffs->values[0], scene_plane_coeffs->values[1],
      scene_plane_coeffs->values[1]);
  if (n.dot(tt_axis) < 0) {
    scene_plane_coeffs->values[0] *= -1.f;
    scene_plane_coeffs->values[1] *= -1.f;
    scene_plane_coeffs->values[2] *= -1.f;
    scene_plane_coeffs->values[3] *= -1.f;
  }

  // plane pointlcoud and hull
  float a(object_init_x), b(object_init_y), c(object_init_z),
      nx(scene_plane_coeffs->values[0]), ny(scene_plane_coeffs->values[1]),
      nz(scene_plane_coeffs->values[2]), d(scene_plane_coeffs->values[3]);
  float t = (-d - (a*nx + b*ny + c*nz)) / (nx*nx + ny*ny + nz*nz);
  PointXYZ p(a+t*nx, b+t*ny, c+t*nz);  // point on the TT plane

  // make grid of points
  PointCloudT::Ptr grid = boost::make_shared<PointCloudT>();
  int N = 100;  // size of grid
  d = 0.05;  // distance between grid points (m)
  for (int x = 0; x < N; x++) {
    for (int y = 0; y < N; y++) {
      PointT g(UINT8_MAX, UINT8_MAX, UINT8_MAX);
      g.x = (x - N/2)*d;
      g.y = (y - N/2)*d;
      g.z = 0.f;
      grid->push_back(g);
    }
  }
  // transform grid to lie on turntable
  tformT T = get_tabletop_rot();
  T(0, 3) = p.x;
  T(1, 3) = p.y;
  T(2, 3) = p.z;
  transformPointCloud(*grid, *grid, T);

  // populate pose of turntable base frame
  T(0, 3) = object_init_x;
  T(1, 3) = object_init_y;
  T(2, 3) = object_init_z;
  T_c_b = T;

  // make convex hull from turntable points
  ConvexHull<PointT> hull;
  hull.setInputCloud(grid);
  hull.reconstruct(*scene_plane_hull_points);
  if (hull.getDimension() != 2) {
    console::print_warn("Estimated plane pointcloud is not 2D.\n");
    return false;
  } else return true;
}


int get_top_point(PointCloud<PointT>::Ptr &cloud) {
  int index = -1;
  float max_z = -FLT_MAX;
  for (int i=0; i < cloud->size(); i++) {
    float z = cloud->at(i).z;
    if (z > max_z) {
      max_z = z;
      index = i;
    }
  }
  return index;
}


// esimates a plane perpendicular to axis
bool PoseEstimator::estimate_perpendicular_plane(const PointCloudT::ConstPtr &in,
                                  const Eigen::Vector3f &axis,
                                  ModelCoefficientsPtr coeffs,
                                  PointCloudT::Ptr plane_cloud,
                                  PointIndicesPtr plane_idx,
                                  float epsAngle,
                                  float inlier_thresh) {
  SACSegmentation<PointT> plane_seg;
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(SACMODEL_PERPENDICULAR_PLANE);
  plane_seg.setMethodType(SAC_RANSAC);
  plane_seg.setDistanceThreshold(inlier_thresh);  // good value for small plane = 3e-3
  plane_seg.setMaxIterations(1e6);
  plane_seg.setAxis(axis);
  plane_seg.setEpsAngle(epsAngle);
  plane_seg.setInputCloud(in);
  PointIndicesPtr plane_idx_ =
      plane_idx ? plane_idx : boost::make_shared<PointIndices>();
  ModelCoefficientsPtr coeffs_ =
      coeffs ? coeffs : boost::make_shared<ModelCoefficients>();
  plane_seg.segment(*plane_idx_, *coeffs_);
  if (plane_idx_->indices.size() == 0) {
    console::print_error("No plane found in the scene.");
    return false;
  }

  if (!plane_cloud) return true;
  ExtractIndices<PointT> extract;
  extract.setInputCloud(in);
  extract.setNegative(false);
  extract.setIndices(plane_idx_);
  extract.filter(*plane_cloud);
  return true;
}


// fits a single plane to the scene and only keeps the inliers
bool PoseEstimator::remove_scene_distortion() {
  // first, find left-facing plane
  Eigen::Vector3f plane_normal(1.f/sqrt(2.f), 1.f/sqrt(2.f), 0);
  float epsAngle(45.f*M_PI/180.f), inlier_thresh(10e-3);
  PointCloudT::Ptr plane_cloud = boost::make_shared<PointCloudT>();
  bool done = estimate_perpendicular_plane(scene_processed, plane_normal,
                               distorted_object_plane_coeffs,
                               plane_cloud, nullptr, epsAngle, inlier_thresh);
  if (distorted_object_plane_coeffs->values[1] < 0) {  //flip
    distorted_object_plane_coeffs->values[0] *= -1;
    distorted_object_plane_coeffs->values[1] *= -1;
    distorted_object_plane_coeffs->values[2] *= -1;
    distorted_object_plane_coeffs->values[3] *= -1;
  }
  float angle = atan2(distorted_object_plane_coeffs->values[1],
      distorted_object_plane_coeffs->values[0]);
  // cout << "Left-facing plane angle = " << angle * 180.f / M_PI << endl;
  if (angle < 70.f*M_PI/180.f && angle > 30.f*M_PI/180.f) {
    copyPointCloud(*plane_cloud, *scene_processed);
    return done;
  }
  // else, estimate the other plane
  plane_normal = Eigen::Vector3f(0, 1, 0);
  return estimate_perpendicular_plane(scene_processed, plane_normal,
                               distorted_object_plane_coeffs,
                               scene_processed, nullptr, epsAngle, inlier_thresh);
}


void PoseEstimator::process_scene() {
  // assume cropping, subsampling and plane estimation is done

  // align the scene and plane inliers to canonical axes (+Z up)
  tformT T = tformT::Identity();
  T(0, 3) = object_init_x;
  T(1, 3) = object_init_y;
  T(2, 3) = object_init_z;
  T = invert_pose(T * get_tabletop_rot());
  PointCloudT::Ptr scene_aligned = boost::make_shared<PointCloudT>();
  PointCloudT::Ptr hull_aligned = boost::make_shared<PointCloudT>();
  transformPointCloud(*scene_cropped_subsampled, *scene_aligned, T);
  transformPointCloud(*scene_plane_hull_points, *hull_aligned, T);

  // segment object sticking out of the plane
  PointIndicesPtr idx = boost::make_shared<PointIndices>();
  ExtractIndices<PointT> extract;
  ExtractPolygonalPrismData<PointT> prism;
  prism.setViewPoint(0, 0, 1);
  prism.setInputCloud(scene_aligned);
  prism.setInputPlanarHull(hull_aligned);
  extract.setInputCloud(scene_aligned);
  extract.setNegative(false);

  prism.setHeightLimits(0, scene_boxsize_z);
  prism.segment(*idx);
  extract.setIndices(idx);
  extract.filter(*scene_processed);

  // segment out the white object
  if (white_thresh > 0) {
    idx->indices.clear();
    for (int i=0; i<scene_processed->size(); i++) {
      const auto &p = scene_processed->at(i);
      float gray = 0.21*p.r + 0.72*p.g + 0.07*p.b;
      if (gray > white_thresh) idx->indices.push_back(i);
    }
    extract.setInputCloud(scene_processed);
    extract.setIndices(idx);
    extract.filter(*scene_processed);
  }

  // fudge factor
  tformT T_fudge = tformT::Identity();
  T_fudge(2, 3) = height_adjust;
  transformPointCloud(*scene_processed, *scene_processed, T_fudge);

  // remove noise
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(scene_processed);
  sor.setMeanK(20);
  sor.setStddevMulThresh(1.5);
  sor.filter(*scene_processed);

  // remove distortion
  if (scene_distorted) remove_scene_distortion();

  // save the segmented object
  copyPointCloud(*scene_processed, *scene_object_segmented);
  transformPointCloud(*scene_object_segmented, *scene_object_segmented,
                      invert_pose(T));

  // measure dimension of object - highly recommended along Z axis!
  PointT min_pt, max_pt;
  getMinMax3D<PointT>(*scene_processed, min_pt, max_pt);
  switch (scale_axis) {
  case 'x':
      axis_size = fabs(max_pt.x - min_pt.x);
      console::print_warn("Scaling by X axis not recommended!");
      break;
  case 'y':
      axis_size = fabs(max_pt.y - min_pt.y);
      console::print_warn("Scaling by Y axis not recommended!");
      break;
  case 'z':
      axis_size = fabs(max_pt.z - min_pt.z);
      break;
  }
  // console::print_info("Scene object dimension along %c axis is %4.3f\n",
  //                     scale_axis, axis_size);
}


void PoseEstimator::set_object_flip_angles(float rx, float ry, float rz) {
  object_flip_angles[0] = rx * M_PI / 180.f;
  object_flip_angles[1] = ry * M_PI / 180.f;
  object_flip_angles[2] = rz * M_PI / 180.f;
  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf(object_flip_angles[2], Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(object_flip_angles[1], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(object_flip_angles[0], Eigen::Vector3f::UnitX());
  object_flip.block<3, 3>(0, 0) = R;
}


void PoseEstimator::process_object() {
  // Assumes the object has it's Z axis pointing up!
  // subsample
  object_vox.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  object_vox.filter(*object_processed);

  // flip object
  pcl::transformPointCloud(*object_processed, *object_processed, object_flip);
  // cout << "Object slid by " << object_flip(0, 3) << ", " << object_flip(1, 3)
  //      << ", " << object_flip(2, 3) << endl;
  // cout << "Object flipped by " << object_flip_angles[0] << " X, "
  //      << object_flip_angles[1] << " Y, " << object_flip_angles[2] << " Z."
  //      << endl;

  if (forced_object_scale < 0.f) {
    // scale by size of object in scene
    PointT min_pt, max_pt;
    getMinMax3D<PointT>(*object_processed, min_pt, max_pt);
    float scale = axis_size;
    switch (scale_axis) {
    case 'x':
      scale /= fabs(max_pt.x - min_pt.x);
      break;
    case 'y':
      scale /= fabs(max_pt.y - min_pt.y);
      break;
    case 'z':
      scale /= fabs(max_pt.z - min_pt.z);
      break;
    }
    object_scale = object_adj_rot = object_adj_pos = tformT::Identity();
    object_scale(0, 0) = object_scale(1, 1) = object_scale(2, 2) = scale;
  } else {
    object_scale(0, 0) = object_scale(1, 1) = object_scale(2, 2) =
        forced_object_scale;
    // cout << "Object scale forced" << endl;
  }
  // cout << "Scaled object by " << object_scale(0, 0) << "x" << endl;
}


// gives rotation of turntable w.r.t. camera
PoseEstimator::tformT PoseEstimator::get_tabletop_rot(Eigen::Vector3f obj_normal) {
  // unit normal to plane
  float a = scene_plane_coeffs->values[0], b = scene_plane_coeffs->values[1],
      c = scene_plane_coeffs->values[2];
  float r = sqrtf(a*a + b*b + c*c);
  Eigen::Vector3f plane_normal = Eigen::Vector3f(a, b, c)/r;
  Eigen::Vector3f v(obj_normal.cross(plane_normal));
  tformT vx = tformT::Zero();
  vx(0, 1) = -v(2); vx(1, 0) = +v(2);
  vx(0, 2) = +v(1); vx(2, 0) = -v(1);
  vx(1, 2) = -v(0); vx(2, 1) = +v(0);
  float dotp = plane_normal.dot(obj_normal);

  return tformT::Identity() + vx + (vx*vx)/(1+dotp);
}


PoseEstimator::tformT PoseEstimator::invert_pose(PoseEstimator::tformT const &in) {
  tformT out = tformT::Identity();
  Eigen::Matrix3f R = in.block<3, 3>(0, 0);
  out.block<3, 3>(0, 0) = R.transpose();
  out.block<3, 1>(0, 3) = -R.transpose() * in.block<3, 1>(0, 3);

  return out;
}


PoseEstimator::tformT PoseEstimator::get_object_pose() {
  tformT Tf = tformT::Identity();
  Tf(2, 3) = -height_adjust;
  tformT T = T_b_f_offset * T_b_f * T_f_o * Tf;
  tformT P = tformT::Identity();
  P.block<3, 1>(0, 3) = T.block<3, 1>(0, 3);
  tformT R = P * object_adj_rot * invert_pose(P);
  T = T_icp * object_adj_pos * R * T;
  return T;
}


PointCloudT::ConstPtr PoseEstimator::get_processed_object() {
  PointCloudT::Ptr out = boost::make_shared<PointCloudT>();
  tformT T = get_object_pose();
  T *= object_scale;
  transformPointCloud(*object_processed, *out, T);
  return out;
}


PointXYZ PoseEstimator::get_minpt_offset() {
  PointT ps_min, ps_max, po_min, po_max;
  getMinMax3D(*scene_processed, ps_min, ps_max);

  PointCloudT::Ptr object_cloud = boost::make_shared<PointCloudT>();
  if (scene_distorted) {
    // need to estimate single plane just like we did for the scene,
    // to avoid translation offset
    Eigen::Vector3f plane_normal(distorted_object_plane_coeffs->values[0],
        distorted_object_plane_coeffs->values[1],
        distorted_object_plane_coeffs->values[2]);
    float epsAngle(30.f*M_PI/180.f);

    // find 2 planes, keep the nearest one
    PointCloudT::Ptr object(boost::make_shared<PointCloudT>()),
        object_rm1(boost::make_shared<PointCloudT>());
    PointCloudT::Ptr plane1_cloud(boost::make_shared<PointCloudT>()),
        plane2_cloud(boost::make_shared<PointCloudT>());
    copyPointCloud(*get_processed_object(), *object);
    ModelCoefficientsPtr coeffs1(boost::make_shared<ModelCoefficients>()),
        coeffs2(boost::make_shared<ModelCoefficients>());
    PointIndicesPtr indices1(boost::make_shared<PointIndices>()),
        indices2(boost::make_shared<PointIndices>());

    // estimate first plane
    estimate_perpendicular_plane(object, plane_normal, coeffs1,
                                 plane1_cloud, indices1, epsAngle);
    // float plane1_y(-coeffs1->values[3]*coeffs1->values[1]);
    float plane1_y(0);
    for (const auto &p: plane1_cloud->points) plane1_y += p.y;
    plane1_y /= plane1_cloud->size();

    // remove first plane
    ExtractIndices<PointT> extract;
    extract.setInputCloud(object);
    extract.setIndices(indices1);
    extract.setNegative(true);
    extract.filter(*object_rm1);
    extract.setNegative(false);

    // estimate second plane
    estimate_perpendicular_plane(object_rm1, plane_normal, coeffs2,
                                 plane2_cloud, indices2, epsAngle);
    // float plane2_y(-coeffs2->values[3]*coeffs2->values[1]);
    float plane2_y(0);
    for (const auto &p: plane2_cloud->points) plane2_y += p.y;
    plane2_y /= plane2_cloud->size();

    // choose the plane with smallest distance to origin
    if (plane1_y > plane2_y) { // plane 2 is closer
      extract.setInputCloud(object_rm1);
      extract.setIndices(indices2);
    } // else, extract is already setup correctly
    extract.filter(*object_cloud);
  } else copyPointCloud(*get_processed_object(), *object_cloud);
  getMinMax3D(*object_cloud, po_min, po_max);
  PointXYZ p(ps_min.x-po_min.x, ps_min.y-po_min.y, ps_min.z-po_min.z);
  return p;
}


// initializes the data for running ICP
void PoseEstimator::init_icp() {
  T_icp = tformT::Identity();
  // object initial azimuth angle
  float s = sin(object_init_azim * float(M_PI)/180),
      c = cos(object_init_azim * float(M_PI)/180);
  object_adj_rot = object_adj_pos = tformT::Identity();
  object_adj_rot(0, 0) = c;
  object_adj_rot(0, 1) = -s;
  object_adj_rot(1, 0) = s;
  object_adj_rot(1, 1) = c;
  PointXYZ offset = get_minpt_offset();
  object_adj_pos(0, 3) = offset.x;
  object_adj_pos(1, 3) = offset.y;
  object_adj_pos(2, 3) = offset.z;
  object_adj_pos(0, 3) += object_init_dx;
  object_adj_pos(1, 3) += object_init_dy;
  object_adj_pos(2, 3) += object_init_dz;
}


// do ICP
float PoseEstimator::do_icp() {
  IterativeClosestPoint_Exposed<PointT, PointT> icp;
  PointCloudT::ConstPtr obj_input = get_processed_object();
  icp.setInputSource(scene_processed);
  icp.setInputTarget(obj_input);

  icp.setRANSACOutlierRejectionThreshold(icp_outlier_rejection_thresh);
  icp.setMaxCorrespondenceDistance(icp_max_corr_distance);
  icp.setMaximumIterations(icp_n_iters);
  icp.setUseReciprocalCorrespondences(icp_use_reciprocal_corr);
  icp.setEuclideanFitnessEpsilon(1e-12);
  icp.setTransformationEpsilon(1e-12);

  // decide the ICP degrees of freedom
  if (icp_only_xy) {
    te_lm->setWarpFunction(warp_xy);
  } else if (icp_no_rollpitch && !icp_symmetric_object) {
    te_lm->setWarpFunction(warp_no_rollpitch);
  } else if (icp_no_rollpitch && icp_symmetric_object) {
    te_lm->setWarpFunction(warp_no_rotation);
  } else if (!icp_no_rollpitch && icp_symmetric_object) {
    te_lm->setWarpFunction(warp_no_azim);
  } else {
    te_lm->setWarpFunction(warp_6d);
  }
  icp.setTransformationEstimation(te_lm);

  PointCloudT::Ptr obj_aligned = boost::make_shared<PointCloudT>();
  icp.align(*obj_aligned);
  if (true) {
    auto tree = icp.getSearchMethodTarget();
    float dist(0.f);
    std::vector<int> idx(1);
    std::vector<float> dists(1);
    for (const auto &p: obj_aligned->points) {
      tree->nearestKSearch(p, 1, idx, dists);
      dist += dists[0];
    }
    dist /= scene_processed->points.size();

    // get final object pose
    tformT T = icp.getFinalTransformation();
    T_icp = invert_pose(T);
    return dist;
  } else {
    console::print_error("ICP did not converge.");
    return -1.f;
  }
}


float PoseEstimator::do_auto_icp() {
  float min_azim(0), min_x(0), min_y(0), min_z(0), min_r(FLT_MAX);
  tformT minT;
  float prev_object_init_azim(object_init_azim), prev_object_init_dx(object_init_dx),
      prev_object_init_dy(object_init_dy), prev_object_init_dz(object_init_dz);
  // cm
  float x_min(prev_object_init_dx - 0.f), x_max(prev_object_init_dx + 0.f),
      x_step(2.f);
  float y_min(prev_object_init_dy - 0.f), y_max(prev_object_init_dy + 0.f),
      y_step(2.f);
  float z_min(prev_object_init_dz - 2.f), z_max(prev_object_init_dz + 2.f),
      z_step(2.f);
  float azim_min = prev_object_init_azim -
      azim_search_step * floor(azim_search_range/2.f/azim_search_step);
  float azim_max = prev_object_init_azim +
      azim_search_step * floor(azim_search_range/2.f/azim_search_step);
  for (float azim=azim_min; azim<=azim_max; azim+=azim_search_step) {
    object_init_azim = azim;
    for (float x=x_min/100.f; x<=x_max/100.f; x+=x_step/100.f) {
      object_init_dx = x;
      for (float y=y_min/100.f; y<=y_max/100.f; y+=y_step/100.f) {
        object_init_dy = y;
        for (float z=z_min/100.f; z<=z_max/100.f; z+=z_step/100.f) {
          object_init_dz = z;
          init_icp();
          float r = do_icp();
          // cout << "Azim = " << azim << ", x = " << x << ", y = " << y
          //      << ", z = " << z << ", residual = " << r << endl;
          if (r < min_r) {
            min_r = r;
            min_azim = azim;
            min_x = x;
            min_y = y;
            min_z = z;
            minT = T_icp;
            // cout << "Minimum so far" << endl;
          }
        }
      }
    }
  }
  // cout << "Minimum Azim = " << min_azim << ", x = " << min_x << ", y = " << min_y
  //      << ", z = " << min_z << ", residual = " << min_r << endl;
  if (min_r > 5e-5) cout << "### WARN: Minimum residual " << min_r << endl;

  object_init_dx = min_x;
  object_init_dy = min_y;
  object_init_dz = min_z;
  object_init_azim = min_azim;
  init_icp();
  T_icp = minT;
  object_init_dx = prev_object_init_dx;
  object_init_dy = prev_object_init_dy;
  object_init_dz = prev_object_init_dz;
  object_init_azim = prev_object_init_azim;
  return min_r;
}

bool PoseEstimator::write_pose_file(std::string pose_filename,
                                    std::string scale_filename) {
  tformT T_c_o = tformT::Identity();
  T_c_o(0, 3) = object_init_x;
  T_c_o(1, 3) = object_init_y;
  T_c_o(2, 3) = object_init_z;
  T_c_o *= get_tabletop_rot();
  T_c_o *= get_object_pose();

  T_f_o = invert_pose(T_b_f) * invert_pose(T_b_f_offset) * invert_pose(T_c_b)
      * T_c_o;

  ofstream f(pose_filename, std::ios_base::app);
  if (!f.is_open()) {
    cout << "Could not open " << pose_filename << " for appending" << endl;
    return false;
  }
  f << T_c_o(0, 3) << " ";
  f << T_c_o(1, 3) << " ";
  f << T_c_o(2, 3) << " ";
  f << T_c_o(0, 0) << " ";
  f << T_c_o(0, 1) << " ";
  f << T_c_o(0, 2) << " ";
  f << T_c_o(1, 0) << " ";
  f << T_c_o(1, 1) << " ";
  f << T_c_o(1, 2) << " ";
  f << T_c_o(2, 0) << " ";
  f << T_c_o(2, 1) << " ";
  f << T_c_o(2, 2) << endl;
  f.close();

  f.open(scale_filename);
  if (!f.is_open()) {
    cout << "Could not open " << scale_filename << " for writing" << endl;
    return false;
  }
  f << object_scale(0, 0) << endl;
  f.close();

  return true;
}

bool PoseEstimator::write_tt_file(std::string tt_base_filename) {
  tformT T = get_tabletop_rot();
  ofstream ofile(tt_base_filename);
  if (!ofile.is_open()) {
    cout << "Could not open " << tt_base_filename << " for writing" << endl;
    return false;
  }
  ofile << object_init_x << " " << object_init_y << " " << object_init_z;
  ofile << " " << T(0, 0);
  ofile << " " << T(0, 1);
  ofile << " " << T(0, 2);
  ofile << " " << T(1, 0);
  ofile << " " << T(1, 1);
  ofile << " " << T(1, 2);
  ofile << " " << T(2, 0);
  ofile << " " << T(2, 1);
  ofile << " " << T(2, 2) << endl;
  ofile << scene_plane_coeffs->values[0] << " ";
  ofile << scene_plane_coeffs->values[1] << " ";
  ofile << scene_plane_coeffs->values[2] << " ";
  ofile << scene_plane_coeffs->values[3];
  ofile.close();

  return true;
}

bool PoseEstimator::write_segmented_object(std::string filename) {
  bool done = io::savePCDFile(filename, *scene_object_segmented) == 0;
  return done;
}

/*
void PoseEstimator::scene_object_mask() {
  // segment object sticking out of the plane
  ExtractPolygonalPrismData<PointT> prism;
  PointIndicesPtr pc_idx = boost::make_shared<PointIndices>();
  prism.setInputCloud(scene);
  prism.setInputPlanarHull(scene_hull_points);
  prism.setHeightLimits(scene_min_height, scene_boxsize_z);
  prism.segment(*pc_idx);

  // mask of the object
  cv::Mat mask = cv::Mat::zeros(scene->height, scene->width, CV_8UC1);
  uchar *ptr = mask.ptr(0);
  if (scene->isOrganized()) {
    for (auto idx : pc_idx->indices) ptr[idx] = 255;
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, strel);
    // strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21));
    // cv::erode(mask, mask, strel);
    stringstream filename;
    filename << scene_data_dir + "/" << object_name << "/" << scene_name
             << "_mask.ppm";
    bool done = cv::imwrite(filename.str(), mask);
    if (!done) {
      console::print_error("Could not write mask to %s\n", filename.str());
    }
  } else {
    console::print_error("Scene point-cloud is not organized!\n");
  }
}
*/
