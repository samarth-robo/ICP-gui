#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "mesh_sample.h"
#include <iostream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <QLineEdit>
#include <QAbstractButton>
#include <QCheckBox>
#include <QFileDialog>
#include <QComboBox>

using namespace pcl;
using namespace std;
namespace bfs = boost::filesystem;

PCLViewer::PCLViewer(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PCLViewer),
  scene_cloud(new PointCloudT()), object_cloud(new PointCloudT()),
  scene_vis(new Vis("scene")), object_vis(new Vis("object")),
  icp_vis(new Vis("ICP")),
  pe(new PoseEstimator()), plane_estimated(false), plane_locked(false),
  scene_processed(false), object_processed(false), icp_initialized(false),
  root_dir("../data/"), scene_filename("scene.pcd") {
  ui->setupUi(this);
  this->setWindowTitle("ICP GUI");

  // init the VTK widgets
  // Scene PCL Viewer
  auto widget = ui->scene_widget;
  auto viewer = scene_vis->get_viewer();
  widget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(widget->GetInteractor(), widget->GetRenderWindow());
  widget->update();
  // Object PCL Viewer
  widget = ui->object_widget;
  viewer = object_vis->get_viewer();
  widget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(widget->GetInteractor(), widget->GetRenderWindow());
  widget->update();
  // ICP PCL Viewer
  widget = ui->icp_widget;
  viewer = icp_vis->get_viewer();
  widget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(widget->GetInteractor(), widget->GetRenderWindow());
  widget->update();

  // make signal-slot connections
  connect(ui->scene_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::scene_leaf_size_changed);
  connect(ui->object_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_leaf_size_changed);
  connect(ui->object_x_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_x_changed);
  connect(ui->object_y_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_y_changed);
  connect(ui->object_z_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_z_changed);
  connect(ui->object_dx_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_dx_changed);
  connect(ui->object_dy_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_dy_changed);
  connect(ui->object_dz_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_dz_changed);
  connect(ui->object_init_azimuth_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_init_azim_changed);
  connect(ui->scene_boxsize_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::scene_boxsize_changed);
  connect(ui->icp_corr_dist_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::icp_corr_dist_changed);
  connect(ui->icp_outlier_dist_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::icp_outlier_dist_changed);
  connect(ui->forced_object_scale_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::forced_object_scale_changed);
  connect(ui->height_adjust_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::height_adjust_changed);
  connect(ui->icp_recip_corr_checkbox, &QCheckBox::stateChanged, this,
          &PCLViewer::icp_recip_corr_clicked);
  connect(ui->icp_no_rotation_checkbox, &QCheckBox::stateChanged, this,
          &PCLViewer::icp_no_rotation_clicked);
  connect(ui->scene_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_process_clicked);
  connect(ui->scene_estimate_plane_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_estimate_plane_clicked);
  connect(ui->scene_save_plane_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_save_plane_clicked);
  connect(ui->object_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::object_process_clicked);
  connect(ui->icp_init_button, &QAbstractButton::clicked, this,
          &PCLViewer::icp_init_clicked);
  connect(ui->icp_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::icp_process_clicked);
  connect(ui->icp_save_button, &QAbstractButton::clicked, this,
          &PCLViewer::icp_save_clicked);
  connect(ui->dir_select_button, &QAbstractButton::clicked, this,
          &PCLViewer::dir_select_clicked);
  connect(ui->scene_select_combo_box,
          static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
          this, &PCLViewer::scene_select_combo_box_activated);

  // fill out various line edits
  QString s;
  s.setNum(pe->get_object_leaf_size());
  ui->object_leaf_size_line_edit->setText(s);
  s.setNum(pe->get_scene_leaf_size());
  ui-> scene_leaf_size_line_edit->setText(s);
  s.setNum(pe->get_scene_boxsize_x());
  ui->scene_boxsize_line_edit->setText(s);
  s.setNum(pe->get_object_init_x());
  ui->object_x_line_edit->setText(s);
  s.setNum(pe->get_object_init_y());
  ui->object_y_line_edit->setText(s);
  s.setNum(pe->get_object_init_z());
  ui->object_z_line_edit->setText(s);
  s.setNum(pe->get_object_init_dx());
  ui->object_dx_line_edit->setText(s);
  s.setNum(pe->get_object_init_dy());
  ui->object_dy_line_edit->setText(s);
  s.setNum(pe->get_object_init_dz());
  ui->object_dz_line_edit->setText(s);
  s.setNum(pe->get_forced_object_scale());
  ui->forced_object_scale_line_edit->setText(s);
  s.setNum(pe->get_height_adjust());
  ui->height_adjust_line_edit->setText(s);
  s.setNum(pe->get_object_init_azim());
  ui->object_init_azimuth_line_edit->setText(s);
  s.setNum(pe->get_icp_outlier_dist());
  ui->icp_outlier_dist_line_edit->setText(s);
  s.setNum(pe->get_icp_corr_dist());
  ui->icp_corr_dist_line_edit->setText(s);
  ui->icp_recip_corr_checkbox->setChecked(pe->get_icp_use_recip_corr());
  ui->icp_no_rotation_checkbox->setChecked(pe->get_icp_no_rotation());
}

PCLViewer::~PCLViewer() {
    delete ui;
}

void PCLViewer::init_viewers() {
  // read the point clouds
  bfs::path cloud_filename = bfs::path(root_dir) / "pointclouds" / scene_filename;
  if (io::loadPCDFile<PointT>(cloud_filename.string(), *scene_cloud) == -1) {
    PCL_ERROR("Could not load file %s\n", cloud_filename.string().c_str());
    return;
  } else {
    scene_vis->removeAllPointClouds();
    scene_vis->removeAllShapes();
    scene_vis->addPointCloud(scene_cloud, "scene");
    pe->set_scene(scene_cloud);
    cout << "Loaded scene of size " << scene_cloud->width << " x "
         << scene_cloud->height << endl;
  }
  cloud_filename = bfs::path(std::getenv("HOME")) / "deepgrasp_data" / "models"
      / (object_name + string(".ply"));
  sample_mesh<PointT>(cloud_filename.string(), object_cloud);
  if (object_cloud->empty()) {
    PCL_ERROR("Could not load file %s\n", cloud_filename.string().c_str());
    return;
  } else {
    object_vis->addPointCloud<PointT>(object_cloud, {1, 0, 0}, "object");
    pe->set_object(object_cloud);
    cout << "Loaded object of size " << object_cloud->width << " x "
         << object_cloud->height << endl;
  }

  // read turntable state
  string scene_id = scene_filename.substr(0, scene_filename.find_first_of('.'));
  bfs::path T_b_f_filename = bfs::path(root_dir) / "poses" /
      (string("tt_frame_") + scene_id + ".txt");
  pe->set_T_b_f(T_b_f_filename.string());
}

void PCLViewer::scene_leaf_size_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_scene_leaf_size(s);
    scene_processed = false;
    cout << "Set scene leaf size to " << s << endl;
  } else {
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
  }
}

void PCLViewer::object_leaf_size_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_leaf_size(s);
    cout << "Set object leaf size to " << s << endl;
  } else {
    cout << "ERROR: wrong object leaf size " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_x_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok) / 100;
  if (ok) {
    pe->set_object_init_x(s);
    scene_processed = false;
    cout << "Set object init X to " << s << endl;
  } else {
    cout << "ERROR: wrong object init X " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_y_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_init_y(s);
    scene_processed = false;
    cout << "Set object init Y to " << s << endl;
  } else {
    cout << "ERROR: wrong object init Y " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_z_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_init_z(s);
    scene_processed = false;
    cout << "Set object init Z to " << s << endl;
  } else {
    cout << "ERROR: wrong object init Z " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_dx_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok) / 100;
  if (ok) {
    pe->set_object_init_dx(s);
    cout << "Set object init dX to " << s << endl;
  } else {
    cout << "ERROR: wrong object init dX " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_dy_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok) / 100;
  if (ok) {
    pe->set_object_init_dy(s);
    cout << "Set object init dY to " << s << endl;
  } else {
    cout << "ERROR: wrong object init dY " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_dz_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok) / 100;
  if (ok) {
    pe->set_object_init_dz(s);
    cout << "Set object init dZ to " << s << endl;
  } else {
    cout << "ERROR: wrong object init dZ " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_azim_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_init_azim(s);
    cout << "Set object init azimuth to " << s << " degrees" << endl;
  } else {
    cout << "ERROR: wrong object init azimuth angle " << t.toStdString() << endl;
  }
}

void PCLViewer::scene_boxsize_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_scene_boxsize_x(s);
    pe->set_scene_boxsize_y(s);
    pe->set_scene_boxsize_z(s);
    scene_processed = false;
    cout << "Set scene box size to " << s << endl;
  } else {
    cout << "ERROR: wrong scene box size " << t.toStdString() << endl;
  }
}

void PCLViewer::icp_outlier_dist_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_icp_outlier_dist(s);
    cout << "Set ICP RANSAC outlier threshold to " << s << endl;
  } else {
    cout << "ERROR: wrong ICP outlier distance " << t.toStdString() << endl;
  }
}

void PCLViewer::icp_corr_dist_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_icp_corr_dist(s);
    cout << "Set ICP max. correspondence distance to " << s << endl;
  } else {
    cout << "ERROR: wrong ICP correspondence distance " << t.toStdString()
         << endl;
  }
}

void PCLViewer::forced_object_scale_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_forced_object_scale(s);
    cout << "Set forced object scale to " << s << endl;
  } else {
    cout << "ERROR: wrong forced object scale " << t.toStdString()
         << endl;
  }
}

void PCLViewer::height_adjust_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_height_adjust(s);
    cout << "Set height adjustment to " << s << endl;
  } else {
    cout << "ERROR: wrong height adjustment " << t.toStdString()
         << endl;
  }
}

void PCLViewer::icp_recip_corr_clicked(int state) {
  switch (state) {
  case Qt::Unchecked:
    pe->set_icp_use_recip_corr(false);
    cout << "ICP will not use reciprocal correspondences" << endl;
    break;
  case Qt::Checked:
    pe->set_icp_use_recip_corr(true);
    cout << "ICP will use reciprocal correspondences" << endl;
    break;
  default:
    cout << "Wrong state of checkbox!" << endl;
  }
}

void PCLViewer::icp_no_rotation_clicked(int state) {
  switch (state) {
  case Qt::Unchecked:
    pe->set_icp_no_rotation(false);
    cout << "ICP will estimate rotation" << endl;
    break;
  case Qt::Checked:
    pe->set_icp_no_rotation(true);
    cout << "ICP will not estimate rotation" << endl;
    break;
  default:
    cout << "Wrong state of checkbox!" << endl;
  }
}

void PCLViewer::scene_estimate_plane_clicked(bool checked) {
  auto min_pt = pe->get_scene_box_min_pt();
  auto max_pt = pe->get_scene_box_max_pt();
  scene_vis->removeAllShapes();
  scene_vis->addCube(min_pt, max_pt);
  PointXYZ p(pe->get_object_init_x(), pe->get_object_init_y(),
             pe->get_object_init_z());
  scene_vis->addSphere(p, 0.005, {1, 0, 1});

  if (!plane_locked) {
    if (!pe->estimate_plane_params()) {
      cout << "WARN: Could not estimate plane" << endl;
      return;
    }
    plane_estimated = true;
    cout << "Plane estimated" << endl;
  } else {
    plane_estimated = true;
    cout << "Previous plane estimate used" << endl;
  }
  auto plane = pe->get_scene_plane_coeffs();
  scene_vis->addPlane(plane, pe->get_object_init_x(),
                      pe->get_object_init_y(), pe->get_object_init_z());

  // draw arrow in direction of plane normal
  PointXYZ p0;
  p0.x = p.x + 0.3 * plane->values[0];
  p0.y = p.y + 0.3 * plane->values[1];
  p0.z = p.z + 0.3 * plane->values[2];
  scene_vis->addArrow(p0, p);

  cout << "Plane and box drawn" << endl;
}

void PCLViewer::scene_process_clicked(bool checked) {
  if (!plane_estimated) {
    cout << "WARN: Estimate plane first!" << endl;
    return;
  }
  pe->process_scene();
  scene_processed = true;
  cout << "scene processed" << endl;
  scene_vis->removeAllPointClouds();
  scene_vis->removeAllShapes();
  scene_vis->addPointCloud(pe->get_processed_scene(), "scene");
}

void PCLViewer::object_process_clicked(bool checked) {
  if (!scene_processed) {
    cout << "WARN: process scene first!" << endl;
    return;
  }
  pe->process_object();
  object_processed = true;
  cout << "object processed" << endl;
  object_vis->removeAllPointClouds();
  object_vis->addPointCloud<PointT>(pe->get_processed_object(), {1, 0, 0},
                                    "object");
  pe->set_object_init_azim(0);
  pe->set_object_init_dx(0);
  pe->set_object_init_dy(0);
  pe->set_object_init_dz(0);
  ui->object_init_azimuth_line_edit->setText("0");
  ui->object_dx_line_edit->setText("0");
  ui->object_dy_line_edit->setText("0");
  ui->object_dz_line_edit->setText("0");
  icp_init_clicked(false);
}

void PCLViewer::refresh_icp_viewer(bool whole_scene) {
  icp_vis->removeAllPointClouds();
  if (whole_scene)
    icp_vis->addPointCloud<PointT>(pe->get_cropped_subsampled_scene(),
    {0, 1, 0}, "scene");
  else
    icp_vis->addPointCloud<PointT>(pe->get_processed_scene(), {0, 1, 0},
                                   "scene");
  icp_vis->addPointCloud<PointT>(pe->get_processed_object(), {1, 0, 0},
                                 "object");
}

void PCLViewer::icp_init_clicked(bool checked) {
  if (scene_processed) {
    if (object_processed) {
      pe->init_icp();
      icp_initialized = true;
      refresh_icp_viewer();
    } else cout << "WARN: process object first!" << endl;
  } else cout << "WARN: process scene first!" << endl;
}

void PCLViewer::icp_process_clicked(bool checked) {
  if (scene_processed) {
    if (object_processed) {
      if (icp_initialized) {
        if (pe->do_auto_icp() > 0) {
          refresh_icp_viewer();
          // object_processed = false;
        }
      } else cout << "WARN: Initialize ICP first!" << endl;
    } else cout << "WARN: process object first!" << endl;
  } else cout << "WARN: process scene first!" << endl;
}

void PCLViewer::icp_save_clicked(bool checked) {
  string scene_name = scene_filename.substr(0, scene_filename.find("."));
  string pose_filename = root_dir + string("/poses/tt_frame_") + scene_name +
      string(".txt");
  string scale_filename = root_dir + string("/scale.txt");
  if (pe->write_pose_file(pose_filename, scale_filename))
    cout << pose_filename << " and " << scale_filename << " written" << endl;
}

void PCLViewer::scene_save_plane_clicked(bool checked) {
  if (plane_estimated) {
    string tt_base_filename = root_dir + string("/poses/tt_base_processed.txt");
    if (pe->write_tt_file(tt_base_filename)) {
      plane_locked = true;
      cout << tt_base_filename << " written" << endl;
    }
  } else console::print_warn("Estimate plane first!");
}

void PCLViewer::dir_select_clicked(bool checked) {
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::Directory);
  dialog.setViewMode(QFileDialog::Detail);
  QStringList dir;
  bool done = false;
  while (!done) {
    if (dialog.exec()) {
      dir = dialog.selectedFiles();
      root_dir = dir[0].toStdString();
      done = true;
    }
  }
  cout << "Root directory set to " << root_dir << endl;

  // read object name
  string filename = root_dir + string("/object_name.txt");
  ifstream f(filename);
  if (f.is_open()) {
    f >> object_name;
    f.close();
  } else {
    console::print_error("Could not open %s for reading", filename.c_str());
    return;
  }
  cout << "Object name is " << object_name << endl;

  // read init XYZ info from txt file
  filename = root_dir + string("/poses/tt_base.txt");
  f.open(filename);
  if (!f.is_open()) {
    console::print_error("Could not open %s for reading\n", filename.c_str());
    return;
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
  QString s;
  s.setNum(T(0, 3));
  ui->object_x_line_edit->setText(s);
  s.setNum(T(1, 3));
  ui->object_y_line_edit->setText(s);
  s.setNum(T(2, 3));
  ui->object_z_line_edit->setText(s);

  // refresh combo-box and populate with list of pcds in directory
  QComboBox *cb = ui->scene_select_combo_box;
  cb->clear();
  bfs::path root(root_dir+string("/pointclouds"));
  if (bfs::is_directory(root)) {
    for (auto it : bfs::directory_iterator(root)) {
      string filename = it.path().filename().string();
      if (filename.find("segmented") != string::npos) continue;
      cb->addItem(QString(filename.c_str()));
      cout << "Found " << filename << endl;
    }
    // load the first pointcloud
    scene_select_combo_box_activated(ui->scene_select_combo_box->itemText(0));
  } else {
    cout << "WARN: 'pointclouds' directory not found in " << root_dir << endl;
    return;
  }

  // read parameters for flipping object
  f.open(root_dir + string("/object_flip.txt"));
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
}

void PCLViewer::scene_select_combo_box_activated(const QString &text) {
  scene_filename = text.toStdString();
  init_viewers();
  plane_estimated = scene_processed = object_processed = icp_initialized = false;
  cout << "Scene set to " << root_dir + string("/pointclouds/") + scene_filename
       << endl;
}
