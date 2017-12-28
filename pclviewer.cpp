#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "mesh_sample.h"
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <QLineEdit>
#include <QAbstractButton>
#include <QCheckBox>

using namespace pcl;
using namespace std;

PCLViewer::PCLViewer(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PCLViewer),
  scene_cloud(new PointCloudT()), object_cloud(new PointCloudT()),
  scene_vis(new Vis("scene")), object_vis(new Vis("object")),
  icp_vis(new Vis("ICP")),
  pe(new PoseEstimator()),
  scene_processed(false), object_processed(false) {
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

  // read the point clouds
  string cloud_filename("/home/samarth/Documents/sandbox/test_ycb_pose/"
                        "locomotive/00000.pcd");
  if (io::loadPCDFile<PointT>(cloud_filename, *scene_cloud) == -1) {
    PCL_ERROR("Could not load file %s\n", cloud_filename);
    return;
  } else {
    scene_vis->addPointCloud(scene_cloud, "scene");
    pe->set_scene(scene_cloud);
    cout << "Loaded scene of size " << scene_cloud->width << " x "
         << scene_cloud->height << endl;
  }
  cloud_filename = string("/home/samarth/catkin_ws/src/deepgrasp_utils/data/"
                          "ycb/locomotive/meshes/tsdf_mesh.ply");
  sample_mesh<PointT>(cloud_filename, object_cloud);
  if (object_cloud->empty()) {
    PCL_ERROR("Could not load file %s\n", cloud_filename);
    return;
  } else {
    object_vis->addPointCloud(object_cloud, "object");
    pe->set_object(object_cloud);
    cout << "Loaded object of size " << object_cloud->width << " x "
         << object_cloud->height << endl;
  }

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
  s.setNum(pe->get_object_init_azim());
  ui->object_init_azimuth_line_edit->setText(s);
  s.setNum(pe->get_icp_outlier_dist());
  ui->icp_outlier_dist_line_edit->setText(s);
  s.setNum(pe->get_icp_corr_dist());
  ui->icp_corr_dist_line_edit->setText(s);
  ui->icp_recip_corr_checkbox->setChecked(pe->get_icp_use_recip_corr());
  ui->icp_estimate_scale_checkbox->setChecked(pe->get_icp_estimate_scale());

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
  connect(ui->scene_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_process_clicked);
  connect(ui->object_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::object_process_clicked);
  connect(ui->icp_init_button, &QAbstractButton::clicked, this,
          &PCLViewer::icp_init_clicked);
  connect(ui->icp_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::icp_process_clicked);
}

PCLViewer::~PCLViewer() {
    delete ui;
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
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
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
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
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
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
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
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
  }
}

void PCLViewer::object_init_azim_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_object_init_azim(s);
    cout << "Set object init azimuth to " << s << " degrees" << endl;
  } else {
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
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
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
  }
}

void PCLViewer::icp_outlier_dist_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_icp_outlier_dist(s);
    cout << "Set ICP RANSAC outlier threshold to " << s << endl;
  } else {
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
  }
}

void PCLViewer::icp_corr_dist_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_icp_corr_dist(s);
    cout << "Set ICP max. correspondence distance to " << s << endl;
  } else {
    cout << "ERROR: wrong scene leaf size " << t.toStdString() << endl;
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

void PCLViewer::scene_process_clicked(bool checked) {
  pe->process_scene();
  scene_processed = true;
  cout << "scene processed" << endl;
  scene_vis->addPointCloud(pe->get_processed_scene(), "scene");
}

void PCLViewer::object_process_clicked(bool checked) {
  if (scene_processed) {
    pe->process_object();
    object_processed = true;
    cout << "object processed" << endl;
    object_vis->addPointCloud(pe->get_processed_object(), "object");
  } else cout << "WARN: process scene first!" << endl;
}

void PCLViewer::refresh_icp_viewer() {
  icp_vis->removeAllPointClouds();
  icp_vis->addPointCloud<PointT>(pe->get_processed_scene(), {0, 1, 0},
                                 "scene");
  icp_vis->addPointCloud<PointT>(pe->get_processed_object(), {1, 0, 0},
                                 "object");
}

void PCLViewer::icp_init_clicked(bool checked) {
  if (scene_processed) {
    if (object_processed) {
      pe->init_icp();
      refresh_icp_viewer();
    } else cout << "WARN: process object first!" << endl;
  } else cout << "WARN: process scene first!" << endl;
}

void PCLViewer::icp_process_clicked(bool checked) {
  pe->do_icp();
  refresh_icp_viewer();
}
