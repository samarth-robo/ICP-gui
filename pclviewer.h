#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QWidget>

#include "pcl_includes.h"
#include "Vis.h"
#include "poseestimator.h"

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
    void scene_leaf_size_changed(const QString &t);
    void object_leaf_size_changed(const QString &t);
    void scene_min_height_changed(const QString &t);
    void object_init_x_changed(const QString &t);
    void object_init_y_changed(const QString &t);
    void object_init_z_changed(const QString &t);
    void object_init_azim_changed(const QString &t);
    void scene_boxsize_changed(const QString &t);
    void icp_corr_dist_changed(const QString &t);
    void icp_outlier_dist_changed(const QString &t);
    void icp_recip_corr_clicked(int state);
    void icp_estimate_scale_clicked(int state);
    void scene_draw_box_clicked(bool checked);
    void scene_process_clicked(bool checked);
    void object_scale_x_clicked(bool checked);
    void object_scale_y_clicked(bool checked);
    void object_scale_z_clicked(bool checked);
    void object_process_clicked(bool checked);
    void icp_init_clicked(bool checked);
    void icp_process_clicked(bool checked);
    void icp_save_clicked(bool checked);

protected:
    boost::shared_ptr<Vis> scene_vis, object_vis, icp_vis;
    PointCloudT::Ptr scene_cloud, object_cloud;
    boost::shared_ptr<PoseEstimator> pe;
    bool scene_processed, object_processed;

private:
    Ui::PCLViewer *ui;
    void refresh_icp_viewer();
};

#endif // PCLVIEWER_H
