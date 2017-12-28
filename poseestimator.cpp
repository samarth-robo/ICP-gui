#include "poseestimator.h"

#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

using namespace pcl;

PoseEstimator::PoseEstimator(PointCloudT::ConstPtr const &scene_,
                             PointCloudT::ConstPtr const &object_) :
  scene_leaf_size(0.005f), object_leaf_size(0.005f), scene_min_height(0.005f),
  scene_boxsize_x(0.25f), scene_boxsize_y(0.25f), scene_boxsize_z(0.25f),
  object_init_x(0.f), object_init_y(0.f), object_init_z(0.f),
  object_init_azim(0.f), object_pose(PoseEstimator::tformT::Identity()),
  icp_n_iters(50), icp_outlier_rejection_thresh(0.002),
  icp_max_corr_distance(0.01), icp_use_reciprocal_corr(false),
  icp_estimate_scale(true) {
  if (scene_) {
    scene = scene_;
    scene_vox.setInputCloud(scene);
  }
  if (object_) {
    object = object_;
    object_vox.setInputCloud(object);
  }

  // init the various shared pointers
  scene_processed    = boost::make_shared<PointCloudT>();
  object_processed   = boost::make_shared<PointCloudT>();
  scene_hull_points  = boost::make_shared<PointCloudT>();
  scene_plane_coeffs = boost::make_shared<ModelCoefficients>();
}

void PoseEstimator::set_scene(const PointCloudT::Ptr &p) {
  scene = p;
  scene_vox.setInputCloud(scene);
}

void PoseEstimator::set_object(const PointCloudT::Ptr &p) {
  object = p;
  object_vox.setInputCloud(object);
}

void PoseEstimator::process_scene() {
  // subsample using voxel grid
  scene_vox.setLeafSize(scene_leaf_size, scene_leaf_size, scene_leaf_size);
  PointCloudT::Ptr subs = boost::make_shared<PointCloudT>();
  scene_vox.filter(*subs);

  // crop box
  CropBox<PointT> box;
  box.setInputCloud(subs);
  Eigen::Vector4f box_size(scene_boxsize_x, scene_boxsize_y,
                           scene_boxsize_z, 0.f);
  box.setMin(-box_size/2);
  box.setMax(box_size/2);
  box.setTranslation(Eigen::Vector3f(object_init_x, object_init_y,
                                     object_init_z));
  PointCloudT::Ptr cbox = boost::make_shared<PointCloudT>();
  box.filter(*cbox);
  // box.filter(*scene_processed);
  // return;

   // estimate the plane
  console::print_info("Estimating plane...");
  PointIndicesPtr plane_inliers = boost::make_shared<PointIndices>();
  SACSegmentation<PointT> plane_seg;
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(SACMODEL_PLANE);
  plane_seg.setMethodType(SAC_RANSAC);
  plane_seg.setDistanceThreshold(0.001);
  // plane_seg.setInputCloud(scene);
  plane_seg.setInputCloud(cbox);
  plane_seg.segment(*plane_inliers, *scene_plane_coeffs);
  if (plane_inliers->indices.size() == 0) {
    console::print_error("No plane found in the scene.");
    return;
  } else console::print_info("done.\n");

  // plane pointlcoud and hull
  PointCloudT::Ptr plane_cloud = boost::make_shared<PointCloudT>();
  ExtractIndices<PointT> extract;
  // extract.setInputCloud(scene);
  extract.setInputCloud(cbox);
  extract.setIndices(plane_inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);
  ConvexHull<PointT> hull;
  hull.setInputCloud(plane_cloud);
  hull.reconstruct(*scene_hull_points);
  if (hull.getDimension() != 2) {
    console::print_error("Estimated plane pointcloud is not 2D.\n");
    return;
  }

  // segment object sticking out of the plane
  ExtractPolygonalPrismData<PointT> prism;
  PointIndicesPtr idx = boost::make_shared<PointIndices>();
  PointCloudT::Ptr obj = boost::make_shared<PointCloudT>();
  prism.setInputCloud(cbox);
  prism.setInputPlanarHull(scene_hull_points);
  prism.setHeightLimits(scene_min_height, scene_boxsize_z);
  prism.segment(*idx);
  extract.setInputCloud(cbox);
  extract.setIndices(idx);
  extract.filter(*obj);

  // transform by object init position
  tformT T = tformT::Identity();
  T(0, 3) = object_init_x;
  T(1, 3) = object_init_y;
  T(2, 3) = object_init_z;
  transformPointCloud(*obj, *scene_processed, invert_pose(T));
}

void PoseEstimator::process_object() {
  // subsample
  object_vox.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  PointCloudT::Ptr subs = boost::make_shared<PointCloudT>();
  object_vox.filter(*subs);

  // orient by tabletop
  tformT T = get_tabletop_rot();
  transformPointCloud(*subs, *object_processed, T);
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
  transformPointCloud(*object_processed, *out, object_pose);
  return out;
}

// initializes the data for running ICP
void PoseEstimator::init_icp() {
  float s = sin(object_init_azim * float(M_PI)/180),
      c = cos(object_init_azim * float(M_PI)/180);
  tformT T = tformT::Identity();
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  object_pose = T;
}

// do ICP
void PoseEstimator::do_icp() {
  IterativeClosestPoint<PointT, PointT> icp;
  typedef registration::TransformationEstimationSVDScale<PointT, PointT> teSVDT;
  teSVDT::Ptr te_svd_scale = boost::make_shared<teSVDT>();
  icp.setInputSource(object_processed);
  icp.setInputTarget(scene_processed);

  icp.setRANSACOutlierRejectionThreshold(icp_outlier_rejection_thresh);
  icp.setMaxCorrespondenceDistance(icp_max_corr_distance);
  icp.setMaximumIterations(icp_n_iters);
  icp.setUseReciprocalCorrespondences(icp_use_reciprocal_corr);
  if (icp_estimate_scale) icp.setTransformationEstimation(te_svd_scale);

  PointCloudT::Ptr obj_aligned = boost::make_shared<PointCloudT>();
  icp.align(*obj_aligned, object_pose);
  if (icp.hasConverged()) {
    object_pose = icp.getFinalTransformation();
    cout << "ICP converged" << endl;
  } else console::print_error("ICP did not converge.");
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
