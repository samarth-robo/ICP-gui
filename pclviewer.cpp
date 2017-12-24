#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <QLineEdit>

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
    scene_vis->addPointCloud(scene_cloud);
    cout << "Loaded scene of size" << scene_cloud->size() << endl;
  }
  cloud_filename = string("/home/samarth/Documents/sandbox/test_ycb_pose/"
                        "locomotive/00000.pcd");
  if (io::loadPCDFile<PointT>(cloud_filename, *object_cloud) == -1) {
    PCL_ERROR("Could not load file %s\n", cloud_filename);
    return;
  } else {
    object_vis->addPointCloud(object_cloud);
    cout << "Loaded scene of size" << object_cloud->size() << endl;
  }

  // make signal-slot connections
  connect(ui->scene_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::scene_leaf_size_changed);
  connect(ui->object_leaf_size_line_edit, &QLineEdit::textEdited, this,
          &PCLViewer::object_leaf_size_changed);

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
