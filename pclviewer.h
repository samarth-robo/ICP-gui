#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QWidget>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

#include "Vis.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class PCLViewer;
}

class PCLViewer : public QWidget
{
    Q_OBJECT

public:
    explicit PCLViewer(QWidget *parent = 0);
    ~PCLViewer();

public slots:
    void process_button_pressed() {};

protected:
    boost::shared_ptr<Vis> vis;
    PointCloudT::Ptr cloud;

    float leaf_size;

private:
    Ui::PCLViewer *ui;
};

#endif // PCLVIEWER_H
