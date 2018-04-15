#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "mesh_sample.h"
#include <iostream>

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
  icp_vis(new Vis("ICP")), object_flipped(false),
  pe(new PoseEstimator()), plane_estimated(false),
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
  init_viewers();

  // make signal-slot connections
  connect(ui->scene_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::scene_leaf_size_changed);
  connect(ui->object_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_leaf_size_changed);
  connect(ui->scene_minheight_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::scene_min_height_changed);
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
  connect(ui->icp_recip_corr_checkbox, &QCheckBox::stateChanged, this,
          &PCLViewer::icp_recip_corr_clicked);
  connect(ui->icp_estimate_scale_checkbox, &QCheckBox::stateChanged, this,
          &PCLViewer::icp_estimate_scale_clicked);
  connect(ui->object_scale_x_radiobutton, &QRadioButton::clicked, this,
          &PCLViewer::object_scale_x_clicked);
  connect(ui->object_scale_y_radiobutton, &QRadioButton::clicked, this,
          &PCLViewer::object_scale_y_clicked);
  connect(ui->object_scale_z_radiobutton, &QRadioButton::clicked, this,
          &PCLViewer::object_scale_z_clicked);
  connect(ui->scene_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_process_clicked);
  connect(ui->scene_estimate_plane_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_estimate_plane_clicked);
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
  s.setNum(pe->get_scene_min_height());
  ui->scene_minheight_line_edit->setText(s);
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
  s.setNum(pe->get_object_init_azim());
  ui->object_init_azimuth_line_edit->setText(s);
  s.setNum(pe->get_icp_outlier_dist());
  ui->icp_outlier_dist_line_edit->setText(s);
  s.setNum(pe->get_icp_corr_dist());
  ui->icp_corr_dist_line_edit->setText(s);
  ui->icp_recip_corr_checkbox->setChecked(pe->get_icp_use_recip_corr());
  ui->icp_estimate_scale_checkbox->setChecked(pe->get_icp_estimate_scale());
  switch (pe->get_scale_axis()) {
   case 'x':
      ui->object_scale_x_radiobutton->setChecked(true);
      break;
   case 'y':
      ui->object_scale_y_radiobutton->setChecked(true);
      break;
   case 'z':
      ui->object_scale_z_radiobutton->setChecked(true);
  }


}

PCLViewer::~PCLViewer() {
    delete ui;
}

void PCLViewer::init_viewers() {
  // read the point clouds
  string cloud_filename = root_dir + string("/pointclouds/") + scene_filename;
  if (io::loadPCDFile<PointT>(cloud_filename, *scene_cloud) == -1) {
    PCL_ERROR("Could not load file %s\n", cloud_filename);
    return;
  } else {
    scene_vis->removeAllPointClouds();
    scene_vis->removeAllShapes();
    scene_vis->addPointCloud(scene_cloud, "scene");
    pe->set_scene(scene_cloud);
    cout << "Loaded scene of size " << scene_cloud->width << " x "
         << scene_cloud->height << endl;
  }
  cloud_filename = root_dir + string("/object.ply");
  sample_mesh<PointT>(cloud_filename, object_cloud);
  if (object_cloud->empty()) {
    PCL_ERROR("Could not load file %s\n", cloud_filename);
    return;
  } else {
    object_vis->addPointCloud<PointT>(object_cloud, {1, 0, 0}, "object");
    pe->set_object(object_cloud);
    cout << "Loaded object of size " << object_cloud->width << " x "
         << object_cloud->height << endl;
  }
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

void PCLViewer::scene_min_height_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_scene_min_height(s);
    scene_processed = false;
    cout << "Set scene min. height to " << s << endl;
  } else {
    cout << "ERROR: wrong height above table " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_x_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
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
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_init_dx(s);
    cout << "Set object init dX to " << s << endl;
  } else {
    cout << "ERROR: wrong object init dX " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_dy_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_init_dy(s);
    cout << "Set object init dY to " << s << endl;
  } else {
    cout << "ERROR: wrong object init dY " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_dz_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
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

void PCLViewer::icp_estimate_scale_clicked(int state) {
  switch (state) {
  case Qt::Unchecked:
    pe->set_icp_estimate_scale(false);
    cout << "ICP will not estimate scale" << endl;
    break;
  case Qt::Checked:
    pe->set_icp_estimate_scale(true);
    cout << "ICP will estimate scale" << endl;
    break;
  default:
    cout << "Wrong state of checkbox!" << endl;
  }
}

void PCLViewer::scene_estimate_plane_clicked(bool checked) {
  if (!pe->estimate_plane_params()) {
    cout << "WARN: Could not estimate plane" << endl;
    return;
  }
  plane_estimated = true;
  auto min_pt = pe->get_scene_box_min_pt();
  auto max_pt = pe->get_scene_box_max_pt();
  scene_vis->removeAllShapes();
  scene_vis->addCube(min_pt, max_pt);
  scene_vis->addPlane(pe->get_scene_plane_coeffs(), pe->get_object_init_x(),
                      pe->get_object_init_y(), pe->get_object_init_z());
  PointXYZ p(pe->get_object_init_x(), pe->get_object_init_y(),
             pe->get_object_init_z());
  scene_vis->addSphere(p, 0.005, {1, 0, 1});
  cout << "Plane estimated, plane and box drawn" << endl;
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

void PCLViewer::object_scale_x_clicked(bool checked) {
    pe->set_scale_axis('x');
    cout << "Scaling object using X axis" << endl;
}

void PCLViewer::object_scale_y_clicked(bool checked) {
    pe->set_scale_axis('y');
    cout << "Scaling object using Y axis" << endl;
}

void PCLViewer::object_scale_z_clicked(bool checked) {
    pe->set_scale_axis('z');
    cout << "Scaling object using Z axis" << endl;
}

void PCLViewer::object_process_clicked(bool checked) {
  if (!scene_processed) {
    cout << "WARN: process scene first!" << endl;
    return;
  }
  float object_scale = 1.f;
  if (object_flipped) {
    string filename = root_dir + string("/scale.txt");
    ifstream f(filename);
    if (!f.is_open()) {
      cout << "Could not open " << filename << " for reading" << endl;
      throw;
    }
    f >> object_scale;
  }
  pe->process_object(object_scale);
  object_processed = true;
  cout << "object processed" << endl;
  object_vis->removeAllPointClouds();
  object_vis->addPointCloud<PointT>(pe->get_processed_object(), {1, 0, 0},
                                    "object");
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
        if (pe->do_icp()) {
          refresh_icp_viewer(true);
          object_processed = false;
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
  string tt_base_filename = root_dir + string("/poses/tt_base.txt");
  if (pe->write_pose_file(pose_filename, scale_filename, tt_base_filename))
    cout << pose_filename << ", " << scale_filename << " and "
         << tt_base_filename << " written" << endl;
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

  // refresh combo-box and populate with list of pcds in directory
  QComboBox *cb = ui->scene_select_combo_box;
  cb->clear();
  bfs::path root(root_dir+string("/pointclouds"));
  if (bfs::is_directory(root)) {
    for (auto it : bfs::directory_iterator(root)) {
      string filename = it.path().filename().string();
      cb->addItem(QString(filename.c_str()));
      cout << "Found " << filename << endl;
    }
    // load the first pointcloud
    scene_select_combo_box_activated(ui->scene_select_combo_box->itemText(0));
  } else {
    cout << "WARN: 'pointclouds' directory not found in " << root_dir << endl;
    return;
  }

  // read init XYZ info from txt file
  ifstream f(root_dir + string("/poses/tt_base.txt"));
  float x, y, z;
  f >> x >> y >> z;
  pe->set_object_init_x(x);
  pe->set_object_init_y(y);
  pe->set_object_init_z(z);
  f.close();
  QString s;
  s.setNum(x);
  ui->object_x_line_edit->setText(s);
  s.setNum(y);
  ui->object_y_line_edit->setText(s);
  s.setNum(z);
  ui->object_z_line_edit->setText(s);

  // see if object has to be flipped
  object_flipped = root_dir.find("flipped") != string::npos;
  pe->set_object_flipped(object_flipped);
}

void PCLViewer::scene_select_combo_box_activated(const QString &text) {
  scene_filename = text.toStdString();
  init_viewers();
  cout << "Scene set to " << root_dir + string("/pointclouds/") + scene_filename
       << endl;
}
