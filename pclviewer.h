#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QWidget>

#include "pcl_includes.h"
#include "Vis.h"
#include "poseestimator.h"
#include <string>

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
    void dir_select_clicked(bool checked);
    void scene_select_combo_box_activated(const QString &text);
    void scene_leaf_size_changed(const QString &t);
    void object_leaf_size_changed(const QString &t);
    void object_init_x_changed(const QString &t);
    void object_init_y_changed(const QString &t);
    void object_init_z_changed(const QString &t);
    void object_init_dx_changed(const QString &t);
    void object_init_dy_changed(const QString &t);
    void object_init_dz_changed(const QString &t);
    void object_init_azim_changed(const QString &t);
    void scene_boxsize_changed(const QString &t);
    void icp_corr_dist_changed(const QString &t);
    void icp_outlier_dist_changed(const QString &t);
    void forced_object_scale_changed(const QString &t);
    void height_adjust_changed(const QString &t);
    void icp_recip_corr_clicked(int state);
    void icp_no_rollpitch_clicked(int state);
    void icp_symmetric_object_clicked(int state);
    void scene_estimate_plane_clicked(bool checked);
    void scene_save_plane_clicked(bool checked);
    void scene_process_clicked(bool checked);
    void object_process_clicked(bool checked);
    void icp_init_clicked(bool checked);
    void icp_process_clicked(bool checked);
    void icp_save_clicked(bool checked);

protected:
    boost::shared_ptr<Vis> scene_vis, object_vis, icp_vis;
    PointCloudT::Ptr scene_cloud, object_cloud;
    boost::shared_ptr<PoseEstimator> pe;
    bool plane_estimated, plane_locked, scene_processed, object_processed,
    icp_initialized;

private:
    Ui::PCLViewer *ui;
    void refresh_icp_viewer(bool whole_scene = false);
    void init_viewers();
    std::string root_dir, scene_filename, object_name;
};

#endif // PCLVIEWER_H
