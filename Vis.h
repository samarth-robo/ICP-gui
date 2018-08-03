//
// Created by samarth on 4/5/17.
//

#ifndef PROJECT_VIS_H
#define PROJECT_VIS_H

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class Vis {
 private:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  unsigned int count;
  bool shown;
  std::string get_next_id(std::string word);
 public:
  Vis(std::string window_name = std::string("PCL Visualizer"));
  bool addPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     std::string id = std::string(), double pt_size = 1.0);
  bool addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                     std::string id = std::string(), double pt_size = 1.0);
  bool addPointCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     std::string id = std::string(), double pt_size = 1.0);

  template <typename PointT>
  bool addPointCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                     std::vector<double> colors, std::string id = std::string(),
                     double pt_size = 1.0);

  template <typename PointT, typename PointNT>
  bool addPointCloudNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                            typename pcl::PointCloud<PointNT>::ConstPtr normals);

  template <typename PointT>
  bool addLine(PointT p0, PointT p1,
               std::vector<double> colors = {1.0, 1.0, 0.0});

  bool addCube(pcl::PointXYZ min_pt, pcl::PointXYZ max_pt,
               std::vector<double> colors = {1.0, 1.0, 1.0});

  template<typename PointT>
  bool addSphere(PointT p, double radius = 1.0,
      std::vector<double> colors = {1, 1, 1});

  bool addText3D(std::string text, pcl::PointXYZ p,
                 std::vector<double> text_colors = {0.0, 1.0, 1.0});

  template <typename PointT>
  bool addCorrespondences(typename pcl::PointCloud<PointT>::ConstPtr source,
                          typename pcl::PointCloud<PointT>::ConstPtr target,
                          pcl::CorrespondencesConstPtr const &corrs,
                          int skip = 20);
  template <typename PointT>
  bool addCorrespondences(typename pcl::PointCloud<PointT>::ConstPtr source,
                          typename pcl::PointCloud<PointT>::ConstPtr target,
                          std::vector<int> const &corrs, int skip = 20);

  bool addPlane(const pcl::ModelCoefficientsConstPtr &coeffs);
  bool addPlane(const pcl::ModelCoefficientsConstPtr &coeffs, double x, double y,
                double z);

  bool addArrow(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2,
                std::vector<double> c = {1, 0, 0});

  pcl::visualization::PCLVisualizer::Ptr get_viewer() {return viewer;}
  bool removeAllPointClouds() {return viewer->removeAllPointClouds();}
  bool removePointCloud(std::string id) {return viewer->removePointCloud(id);}
  bool removeAllShapes() {return viewer->removeAllShapes();}
  void show(bool block = true);
};

#endif //PROJECT_VIS_H
