#include "poseestimator.h"

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

#include <fstream>

using namespace pcl;

PoseEstimator::PoseEstimator(PointCloudT::ConstPtr const &scene_,
                             PointCloudT::ConstPtr const &object_) :
  scene_leaf_size(0.001f), object_leaf_size(0.001f), scene_min_height(0.004f),
  scene_boxsize_x(0.2f), scene_boxsize_y(0.25f), scene_boxsize_z(0.25f),
  object_init_x(0.f), object_init_y(0.f), object_init_z(0.f),
  object_init_azim(0.f), object_pose(PoseEstimator::tformT::Identity()),
  icp_n_iters(2000), icp_outlier_rejection_thresh(0.005),
  icp_max_corr_distance(0.005), icp_use_reciprocal_corr(false),
  icp_estimate_scale(false), scale_axis('z'),
  object_azim(PoseEstimator::tformT::Identity()),
  object_scale(PoseEstimator::tformT::Identity()), object_init_dx(0),
  object_init_dy(0), object_init_dz(0) {
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
  scene_plane_coeffs       = boost::make_shared<ModelCoefficients>();
}

void PoseEstimator::set_scene(const PointCloudT::Ptr &p) {
  scene = p;
  scene_vox.setInputCloud(scene);
}

void PoseEstimator::set_object(const PointCloudT::Ptr &p) {
  object = p;
  object_vox.setInputCloud(object);
  object_pose = tformT::Identity();
  object_azim = tformT::Identity();
  object_scale = tformT::Identity();
}

PointXYZ PoseEstimator::get_scene_box_min_pt() {
    PointXYZ p;
    p.x = object_init_x - scene_boxsize_x/2;
    // p.y = object_init_y - scene_boxsize_y;
    p.y = object_init_y - 0.15;
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
  box.setMin(Eigen::Vector4f(-scene_boxsize_x/2, -0.15, -scene_boxsize_z/2, 0));
  box.setMax(Eigen::Vector4f(+scene_boxsize_x/2, 0.05, +scene_boxsize_z/2, 0));
  box.setTranslation(Eigen::Vector3f(object_init_x, object_init_y,
                                     object_init_z));
  box.filter(*scene_cropped_subsampled);
}

bool PoseEstimator::estimate_plane_params() {
  // crop and subsample scene
  crop_subsample_scene();

  // estimate the plane
  console::print_info("Estimating plane...");
  PointIndicesPtr plane_inliers = boost::make_shared<PointIndices>();
  SACSegmentation<PointT> plane_seg;
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(SACMODEL_PLANE);
  plane_seg.setMethodType(SAC_RANSAC);
  plane_seg.setDistanceThreshold(0.003);
  plane_seg.setInputCloud(scene_cropped_subsampled);
  plane_seg.segment(*plane_inliers, *scene_plane_coeffs);
  if (plane_inliers->indices.size() == 0) {
    console::print_error("No plane found in the scene.");
    return false;
  } else console::print_info("done.\n");

  // plane pointlcoud and hull
  PointCloudT::Ptr plane_cloud = boost::make_shared<PointCloudT>();
  ExtractIndices<PointT> extract;
  extract.setInputCloud(scene_cropped_subsampled);
  extract.setIndices(plane_inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);
  ConvexHull<PointT> hull;
  hull.setInputCloud(plane_cloud);
  hull.reconstruct(*scene_plane_hull_points);
  if (hull.getDimension() != 2) {
    console::print_error("Estimated plane pointcloud is not 2D.\n");
    return false;
  } else return true;
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
  transformPointCloud(*scene_cropped_subsampled, *scene_aligned, T);
  transformPointCloud(*scene_plane_hull_points, *scene_plane_hull_points, T);

  // segment object sticking out of the plane
  ExtractPolygonalPrismData<PointT> prism;
  prism.setViewPoint(0, 0, 1);
  PointIndicesPtr idx = boost::make_shared<PointIndices>();
  prism.setInputCloud(scene_aligned);
  prism.setInputPlanarHull(scene_plane_hull_points);
  ExtractIndices<PointT> extract;
  extract.setInputCloud(scene_aligned);
  extract.setNegative(false);

  prism.setHeightLimits(scene_min_height, scene_boxsize_z);
  prism.segment(*idx);
  extract.setIndices(idx);
  extract.filter(*scene_processed);

  // remove noise
  // StatisticalOutlierRemoval<PointT> sor;
  // sor.setInputCloud(obj);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(1.0);
  // PointCloudT::Ptr obj_filt = boost::make_shared<PointCloudT>();
  // sor.filter(*obj_filt);

  // measure dimension of object - highly recommended along Z axis!
  PointT min_pt, max_pt;
  getMinMax3D<PointT>(*scene_processed, min_pt, max_pt);
  switch (scale_axis) {
  case 'x':
      axis_size = fabs(max_pt.x - min_pt.x);
      break;
  case 'y':
      axis_size = fabs(max_pt.y - min_pt.y);
      break;
  case 'z':
      axis_size = fabs(max_pt.z - min_pt.z);
      break;
  }
  console::print_info("Scene object dimension along %c axis is %4.3f\n",
                      scale_axis, axis_size);
}

void PoseEstimator::process_object() {
  // Assumes the object has it's Z axis pointing up!
  // subsample
  object_vox.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  object_vox.filter(*object_processed);

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
  object_scale = tformT::Identity();
  object_scale(0, 0) = object_scale(1, 1) = object_scale(2, 2) = scale;
  cout << "Scaled object by " << scale << "x" << endl;
}

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

PointCloudT::ConstPtr PoseEstimator::get_processed_object() {
  PointCloudT::Ptr out = boost::make_shared<PointCloudT>();
  tformT T = object_pose * object_azim * object_scale;
  transformPointCloud(*object_processed, *out, T);
  return out;
}

// initializes the data for running ICP
void PoseEstimator::init_icp() {
  // object initial azimuth angle
  float s = sin(object_init_azim * float(M_PI)/180),
      c = cos(object_init_azim * float(M_PI)/180);
  object_azim = tformT::Identity();
  object_azim(0, 0) = c;
  object_azim(0, 1) = -s;
  object_azim(1, 0) = s;
  object_azim(1, 1) = c;
  object_pose = tformT::Identity();
  object_pose(0, 3) = object_init_dx;
  object_pose(1, 3) = object_init_dy;
  object_pose(2, 3) = object_init_dz;
}

// do ICP
bool PoseEstimator::do_icp() {
  IterativeClosestPoint<PointT, PointT> icp;
  typedef registration::TransformationEstimationSVDScale<PointT, PointT> teSVDT;
  teSVDT::Ptr te_svd_scale = boost::make_shared<teSVDT>();
  PointCloudT::Ptr obj_input = boost::make_shared<PointCloudT>();
  transformPointCloud(*object_processed, *obj_input,
                      object_pose*object_azim*object_scale);
  icp.setInputSource(obj_input);
  icp.setInputTarget(scene_processed);

  icp.setRANSACOutlierRejectionThreshold(icp_outlier_rejection_thresh);
  icp.setMaxCorrespondenceDistance(icp_max_corr_distance);
  icp.setMaximumIterations(icp_n_iters);
  icp.setUseReciprocalCorrespondences(icp_use_reciprocal_corr);
  if (icp_estimate_scale) icp.setTransformationEstimation(te_svd_scale);

  PointCloudT::Ptr obj_aligned = boost::make_shared<PointCloudT>();
  icp.align(*obj_aligned);
  if (icp.hasConverged()) {
    cout << "ICP converged" << endl;

    // get final object pose
    tformT T = icp.getFinalTransformation();
    object_pose = T * object_pose;
    T = tformT::Identity();
    T(0, 3) = object_init_x;
    T(1, 3) = object_init_y;
    T(2, 3) = object_init_z;
    T *= get_tabletop_rot();
    object_pose = T * object_pose;
    return true;
  } else {
    console::print_error("ICP did not converge.");
    return false;
  }
}

bool PoseEstimator::write_pose_file(std::string pose_filename,
                                    std::string scale_filename,
                                    std::string tt_base_filename) {
  tformT T = object_pose * object_azim;
  Eigen::Quaternionf q(T.block<3, 3>(0, 0));

  ofstream f(pose_filename, std::ios_base::app);
  if (!f.is_open()) {
    cout << "Could not open " << pose_filename << " for appending" << endl;
    return false;
  }
  f << T(0, 3) << " ";
  f << T(1, 3) << " ";
  f << T(2, 3) << " ";
  f << T(0, 0) << " ";
  f << T(0, 1) << " ";
  f << T(0, 2) << " ";
  f << T(1, 0) << " ";
  f << T(1, 1) << " ";
  f << T(1, 2) << " ";
  f << T(2, 0) << " ";
  f << T(2, 1) << " ";
  f << T(2, 2) << endl;
  // f << T(0, 3) << " " << T(1, 3) << " " << T(2, 3) << " " << q.w() << " "
  //   << q.x() << " " << q.y() << " " << q.z() << endl;
  f.close();

  f.open(scale_filename);
  if (!f.is_open()) {
    cout << "Could not open " << scale_filename << " for writing" << endl;
    return false;
  }
  f << object_scale(0, 0) << endl;
  f.close();

  ifstream ifile(tt_base_filename);
  if (!ifile.is_open()) {
    cout << "Could not open " << tt_base_filename << " for reading" << endl;
    return false;
  }
  float x, y, z;
  ifile >> x >> y >> z;
  ifile.close();

  T = get_tabletop_rot();
  ofstream ofile(tt_base_filename);
  if (!ofile.is_open()) {
    cout << "Could not open " << tt_base_filename << " for writing" << endl;
    return false;
  }
  ofile << x << " " << y << " " << z;
  ofile << " " << T(0, 0);
  ofile << " " << T(0, 1);
  ofile << " " << T(0, 2);
  ofile << " " << T(1, 0);
  ofile << " " << T(1, 1);
  ofile << " " << T(1, 2);
  ofile << " " << T(2, 0);
  ofile << " " << T(2, 1);
  ofile << " " << T(2, 2);
  ofile.close();

  return true;
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
