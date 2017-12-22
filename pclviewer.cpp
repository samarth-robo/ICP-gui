#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <iostream>

#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace std;

PCLViewer::PCLViewer(QWidget *parent) : QWidget(parent), ui(new Ui::PCLViewer),
    cloud(new PointCloudT()), vis(new Vis("Qt Viewer"))
{
    ui->setupUi(this);
    this->setWindowTitle("PCL Viewer");

    // read the point cloud
    string cloud_filename("/home/samarth/Documents/sandbox/test_ycb_pose/locomotive/00000.pcd");
    if (io::loadPCDFile<PointT>(cloud_filename, *cloud) == -1) {
        PCL_ERROR("Could not load file %s\n", cloud_filename);
        return;
    } else {
        cout << "Loaded point cloud of size" << cloud->size() << endl;
    }

    // setup the QVTK window
    const pcl::visualization::PCLVisualizer::Ptr viewer = vis->get_viewer();
    ui->widget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->widget->GetInteractor(), ui->widget->GetRenderWindow());
    ui->widget->update();
}

PCLViewer::~PCLViewer()
{
    delete ui;
}
