#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <QLineEdit>
#include <QAbstractButton>

using namespace pcl;
using namespace std;

PCLViewer::PCLViewer(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PCLViewer),
  scene_cloud(new PointCloudT()), object_cloud(new PointCloudT()),
  scene_vis(new Vis("scene")), object_vis(new Vis("object")),
  pe(new PoseEstimator()) {
  ui->setupUi(this);
  this->setWindowTitle("ICP GUI");

  // init the widgets
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

  // various line edits
  QString s;
  s.setNum(pe->get_object_leaf_size());
  ui->object_leaf_size_line_edit->setText(s);
  s.setNum(pe->get_scene_leaf_size());
  ui-> scene_leaf_size_line_edit->setText(s);

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
  cloud_filename = string("/home/samarth/Documents/sandbox/test_ycb_pose/"
                        "locomotive/00000.pcd");
  if (io::loadPCDFile<PointT>(cloud_filename, *object_cloud) == -1) {
    PCL_ERROR("Could not load file %s\n", cloud_filename);
    return;
  } else {
    object_vis->addPointCloud(object_cloud, "object");
    pe->set_object(object_cloud);
    cout << "Loaded scene of size " << object_cloud->width << " x "
         << object_cloud->height << endl;
  }

  // make signal-slot connections
  connect(ui->scene_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::scene_leaf_size_changed);
  connect(ui->object_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_leaf_size_changed);
  connect(ui->scene_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::scene_process_clicked);
  connect(ui->object_process_button, &QAbstractButton::clicked, this,
          &PCLViewer::object_process_clicked);

}

PCLViewer::~PCLViewer() {
    delete ui;
}

void PCLViewer::scene_leaf_size_changed(const QString &t) {
  bool ok = false;
  float s = t.toFloat(&ok);
  if (ok) {
    pe->set_scene_leaf_size(s);
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

void PCLViewer::scene_process_clicked(bool checked) {
  auto processed_scene = pe->process_scene();
  cout << "scene processed" << endl;
  scene_vis->addPointCloud(processed_scene, "scene");
}

void PCLViewer::object_process_clicked(bool checked) {
  auto processed_object = pe->process_object();
  cout << "object processed" << endl;
  object_vis->addPointCloud(processed_object, "object");
}
