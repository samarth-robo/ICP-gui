#include "poseestimator.h"

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/registration.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <fstream>

using namespace pcl;

PoseEstimator::PoseEstimator(PointCloudT::ConstPtr const &scene_,
                             PointCloudT::ConstPtr const &object_) :
  scene_leaf_size(0.001f), object_leaf_size(0.001f),
  scene_boxsize_x(0.3f), scene_boxsize_y(0.3f), scene_boxsize_z(0.3f),
  object_init_x(0.f), object_init_y(0.f), object_init_z(0.f),
  object_init_azim(0.f),
  object_pose(PoseEstimator::tformT::Identity()),
  icp_n_iters(40), icp_outlier_rejection_thresh(0.005),
  icp_max_corr_distance(0.05), icp_use_reciprocal_corr(false),
  icp_no_rotation(false),
  scale_axis('z'),
  object_azim(PoseEstimator::tformT::Identity()),
  height_adjust(0.000),
  object_scale(PoseEstimator::tformT::Identity()),
  object_init_dx(0), object_init_dy(0), object_init_dz(0),
  forced_object_scale(1.f),
  T_f_o(PoseEstimator::tformT::Identity()),
  white_thresh(150.f) {
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
  // crop and subsample scene
  crop_subsample_scene();
}

void PoseEstimator::set_object(const PointCloudT::Ptr &p) {
  object = p;
  object_vox.setInputCloud(object);
  object_pose = tformT::Identity();
  object_azim = tformT::Identity();
  object_scale = tformT::Identity();
  object_flip = tformT::Identity();
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

  return true;
}

bool PoseEstimator::estimate_plane_params() {
  // estimate the plane
  console::print_info("Estimating plane...");
  PointIndicesPtr plane_inliers = boost::make_shared<PointIndices>();
  SACSegmentation<PointT> plane_seg;
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(SACMODEL_PERPENDICULAR_PLANE);
  plane_seg.setMethodType(SAC_RANSAC);
  plane_seg.setDistanceThreshold(0.003);  // good value for small plane = 3e-3
  plane_seg.setMaxIterations(1e6);
  plane_seg.setAxis(tt_axis);
  plane_seg.setEpsAngle(5 * M_PI/180.0);
  plane_seg.setInputCloud(scene_cropped_subsampled);
  plane_seg.segment(*plane_inliers, *scene_plane_coeffs);
  if (plane_inliers->indices.size() == 0) {
    console::print_error("No plane found in the scene.");
    return false;
  } else console::print_info("done.\n");

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
  ExtractPolygonalPrismData<PointT> prism;
  PointIndicesPtr idx = boost::make_shared<PointIndices>();
  prism.setViewPoint(0, 0, 1);
  prism.setInputCloud(scene_aligned);
  prism.setInputPlanarHull(hull_aligned);
  ExtractIndices<PointT> extract;
  extract.setInputCloud(scene_aligned);
  extract.setNegative(false);

  prism.setHeightLimits(0, scene_boxsize_z);
  prism.segment(*idx);
  extract.setIndices(idx);
  extract.filter(*scene_processed);

  // segment out the white object
  idx->indices.clear();
  for (int i=0; i<scene_processed->size(); i++) {
    const auto &p = scene_processed->at(i);
    float gray = 0.21*p.r + 0.72*p.g + 0.07*p.b;
    if (gray > white_thresh) idx->indices.push_back(i);
  }
  extract.setInputCloud(scene_processed);
  extract.setIndices(idx);
  extract.filter(*scene_processed);

  // fudge factor
  T = tformT::Identity();
  T(2, 3) = height_adjust;
  transformPointCloud(*scene_processed, *scene_processed, T);

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
  console::print_info("Scene object dimension along %c axis is %4.3f\n",
                      scale_axis, axis_size);
}

void PoseEstimator::process_object() {
  // Assumes the object has it's Z axis pointing up!
  // subsample
  object_vox.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  object_vox.filter(*object_processed);

  tformT T = tformT::Identity();
  T.block<3, 1>(0, 3) = object_slide;
  transformPointCloud(*object_processed, *object_processed, T);
  cout << "Object slid by " << object_slide[0] << ", " << object_slide[1]
       << ", " << object_slide[2] << endl;

  // object flip
  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf(object_flip_angles[0], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(object_flip_angles[1], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(object_flip_angles[2], Eigen::Vector3f::UnitZ());
  object_flip.block<3, 3>(0, 0) = R;
  cout << "Object flipped by " << object_flip_angles[0] << " X, "
       << object_flip_angles[1] << " Y, " << object_flip_angles[2] << " Z."
       << endl;

  if (forced_object_scale < 0.f) {
    PointCloudT::Ptr object_flipped = boost::make_shared<PointCloudT>();
    transformPointCloud(*object_processed, *object_flipped, object_flip);
    // scale by size of object in scene
    PointT min_pt, max_pt;
    getMinMax3D<PointT>(*object_flipped, min_pt, max_pt);
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
    object_scale = object_pose = object_azim = tformT::Identity();
    object_scale(0, 0) = object_scale(1, 1) = object_scale(2, 2) = scale;
  } else {
    object_scale(0, 0) = object_scale(1, 1) = object_scale(2, 2) =
        forced_object_scale;
    cout << "Object scale forced" << endl;
  }
  cout << "Scaled object by " << object_scale(0, 0) << "x" << endl;
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

PointCloudT::ConstPtr PoseEstimator::get_processed_object() {
  PointCloudT::Ptr out = boost::make_shared<PointCloudT>();
  tformT Tf = tformT::Identity();
  Tf(2, 3) = -height_adjust;
  tformT T = object_pose * Tf * object_azim * object_flip * object_scale;
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
  object_pose = T_b_f * T_f_o;
  object_pose(0, 3) += object_init_dx;
  object_pose(1, 3) += object_init_dy;
  object_pose(2, 3) += object_init_dz;
}

// do ICP
float PoseEstimator::do_icp() {
  IterativeClosestPoint_Exposed<PointT, PointT> icp;
  PointCloudT::Ptr obj_input = boost::make_shared<PointCloudT>();
  transformPointCloud(*object_processed, *obj_input,
                      object_pose*object_azim*object_flip*object_scale);
  icp.setInputSource(scene_processed);
  icp.setInputTarget(obj_input);

  icp.setRANSACOutlierRejectionThreshold(icp_outlier_rejection_thresh);
  icp.setMaxCorrespondenceDistance(icp_max_corr_distance);
  icp.setMaximumIterations(icp_n_iters);
  icp.setUseReciprocalCorrespondences(icp_use_reciprocal_corr);
  icp.setEuclideanFitnessEpsilon(1e-12);
  icp.setTransformationEpsilon(1e-12);

  PointCloudT::Ptr obj_aligned = boost::make_shared<PointCloudT>();
  icp.align(*obj_aligned);
  tformT guess = tformT::Identity();
  if (true) {
    cout << "ICP converged" << endl;

    auto tree = icp.getSearchMethodTarget();
    float dist(0.f);
    std::vector<int> idx(1);
    std::vector<float> dists(1);
    for (const auto &p: scene_processed->points) {
      tree->nearestKSearch(p, 1, idx, dists);
      dist += dists[0];
    }
    dist /= scene_processed->points.size();
    cout << "Energy = " << dist << endl;

    // get final object pose
    tformT T = icp.getFinalTransformation();
    if (icp_no_rotation) {
      T.block<3,3>(0, 0) = Eigen::Matrix3f::Identity();
    }
    object_pose = invert_pose(T) * object_pose;
    return dist;
  } else {
    console::print_error("ICP did not converge.");
    return -1.f;
  }
}

float PoseEstimator::do_auto_icp() {
  float min_azim, min_x, min_y, min_z, min_r(FLT_MAX);
  tformT minT;
  for (float azim=0; azim<360.f; azim+=30.f) {
    object_init_azim = azim;
    for (float x=-0.f; x<=0.f; x+=1.f) {
      object_init_dx = x;
      for (float y=-0.f; y<=0.f; y+=1.f) {
        object_init_dy = y;
        for (float z=-0.f; z<=0.f; z+=1.f) {
          object_init_dz = z;
          init_icp();
          float r = do_icp();
          cout << "Azim = " << azim << ", x = " << x << ", y = " << y
               << ", z = " << z << ", residual = " << r << endl;
          if (r < min_r) {
            min_r = r;
            min_azim = azim;
            min_x = x;
            min_y = y;
            min_z = z;
            minT = object_pose;
            cout << "Minimum so far" << endl;
          }
        }
      }
    }
  }

  object_init_dx = min_x;
  object_init_dy = min_y;
  object_init_dz = min_z;
  object_pose = minT;
  object_init_azim = min_azim;
  object_azim = tformT::Identity();
  float s = sin(object_init_azim * float(M_PI)/180),
      c = cos(object_init_azim * float(M_PI)/180);
  object_azim(0, 0) = c;
  object_azim(0, 1) = -s;
  object_azim(1, 0) = s;
  object_azim(1, 1) = c;
  return min_r;
}

bool PoseEstimator::write_pose_file(std::string pose_filename,
                                    std::string scale_filename) {
  tformT T_c_o = tformT::Identity();
  T_c_o(0, 3) = object_init_x;
  T_c_o(1, 3) = object_init_y;
  T_c_o(2, 3) = object_init_z;
  T_c_o *= get_tabletop_rot();
  T_c_o *= object_pose * object_azim * object_flip;

  T_f_o = invert_pose(T_b_f) * invert_pose(T_c_b) * T_c_o *
      invert_pose(object_flip);

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
