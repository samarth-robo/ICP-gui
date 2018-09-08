#include "grasp_processor.h"
#include "mesh_sample.h"
#include <fstream>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

GraspProcessor::GraspProcessor(string data_dir) :
  data_dir(data_dir),
  scene_cloud(new PointCloudT()),
  object_cloud(new PointCloudT()),
  pe(new PoseEstimator()),
  plane_locked(false) {}

bool GraspProcessor::process_grasp(string object_name, string session_name) {
  // paths
  bfs::path base_dir = data_dir / session_name / object_name;
  bfs::path pc_dir = base_dir / "pointclouds";

  // read the names of views to be processed
  vector<bfs::path> pc_filenames;
  bfs::path views_filename = base_dir / "views.txt";
  ifstream f(views_filename.string());
  if (!f.is_open()) {
    PCL_ERROR("Could not open %s for reading\n", views_filename.string().c_str());
  }
  string pc_id;
  while (f >> pc_id) {
    bfs::path pc_filename = pc_dir / (pc_id + ".pcd");
    pc_filenames.push_back(pc_filename);
  }
  f.close();

  // read object name
  bfs::path object_name_filename = base_dir / "object_name.txt";
  string real_object_name;
  f.open(object_name_filename.string());
  if (f.is_open()) {
    f >> real_object_name;
    f.close();
  } else {
    PCL_ERROR("Could not open %s for reading\n",
              object_name_filename.string().c_str());
    return false;
  }
  cout << "Object name is " << real_object_name << endl;

  // read the object pointcloud
  bfs::path cloud_filename = bfs::path(std::getenv("HOME")) / "deepgrasp_data"
      / "models" / (real_object_name + string(".ply"));
  sample_mesh<PointT>(cloud_filename.string(), object_cloud);
  if (object_cloud->empty()) {
    PCL_ERROR("Could not load file %s\n", cloud_filename.string().c_str());
    return false;
  } else {
    pe->set_object(object_cloud);
    cout << "Loaded object of size " << object_cloud->width << " x "
         << object_cloud->height << endl;
  }

  // read init XYZ info from txt file
  bfs::path tt_base_filename = base_dir / "poses" / "tt_base.txt";
  f.open(tt_base_filename.string());
  if (!f.is_open()) {
    console::print_error("Could not open %s for reading\n",
                         tt_base_filename.string().c_str());
    return false;
  }
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  float x;
  f >> x; T(0, 3) = x;
  f >> x; T(1, 3) = x;
  f >> x; T(2, 3) = x;
  f >> x; T(0, 0) = x;
  f >> x; T(0, 1) = x;
  f >> x; T(0, 2) = x;
  f >> x; T(1, 0) = x;
  f >> x; T(1, 1) = x;
  f >> x; T(1, 2) = x;
  f >> x; T(2, 0) = x;
  f >> x; T(2, 1) = x;
  f >> x; T(2, 2) = x;
  f.close();
  pe->set_tt_pose(T);

  // read parameters for flipping object
  bfs::path flip_filename = base_dir / "object_flip.txt";
  f.open(flip_filename.string());
  float object_flip_x(0), object_flip_y(0), object_flip_z(0);
  float object_slide_x(0), object_slide_y(0), object_slide_z(0);
  if (!f.is_open())
      cout << "WARN: could not read flip parameters, setting to 0" << endl;
  f >> object_slide_x >> object_slide_y >> object_slide_z
      >> object_flip_x >> object_flip_y >> object_flip_z;
  // degrees
  pe->set_object_flip_angles(object_flip_x, object_flip_y, object_flip_z);
  // cm
  pe->set_object_slide(object_slide_x, object_slide_y, object_slide_z);

  // process the views
  for (const auto &p: pc_filenames) {
    if (!process_view(p, base_dir)) {
      PCL_ERROR("Error processing view %s\n", p.string().c_str());
      return false;
    }
  }
  return true;
}

bool GraspProcessor::process_view(bfs::path pc_filename, bfs::path base_dir) {
  if (io::loadPCDFile<PointT>(pc_filename.string(), *scene_cloud) == -1) {
    PCL_ERROR("Could not load file %s\n", pc_filename.string().c_str());
    return false;
  } else {
    pe->set_scene(scene_cloud);
    cout << "Loaded scene of size " << scene_cloud->width << " x "
         << scene_cloud->height << " from " << pc_filename.string() << endl;
  }

  // read turntable state
  string s(pc_filename.stem().string());
  string scene_id = s.substr(0, s.find_first_of('.'));
  bfs::path T_b_f_filename = base_dir / "poses"
      / (string("tt_frame_") + scene_id + ".txt");
  pe->set_T_b_f(T_b_f_filename.string());

  // estimate plane
  bool plane_estimated(false);
  if (!plane_locked) {
    if (!pe->estimate_plane_params()) {
      cout << "WARN: Could not estimate plane" << endl;
      return false;
    }
    plane_estimated = true;
    cout << "Plane estimated" << endl;
    bfs::path tt_base_filename = base_dir / "poses" / "tt_base.txt";
    if (pe->write_tt_file(tt_base_filename.string())) {
      plane_locked = true;
      cout << tt_base_filename.string() << " written" << endl;
    } else {
      cout << "Could not open " << tt_base_filename << " for writing" << endl;
      return false;
    }
  } else {
    plane_estimated = true;
    cout << "Previous plane estimate used" << endl;
  }

  pe->process_scene();
  pe->process_object();
  pe->set_object_init_azim(0);
  pe->set_object_init_dx(0);
  pe->set_object_init_dy(0);
  pe->set_object_init_dz(0);
  pe->do_auto_icp();
  bfs::path pose_filename = base_dir / "poses" /
      (string("tt_frame_") + scene_id + ".txt");
  bfs::path scale_filename = base_dir / "scale.txt";
  if (pe->write_pose_file(pose_filename.string(), scale_filename.string())) {
    cout << pose_filename << " and " << scale_filename << " written" << endl;
  } else return false;
  return true;
}
