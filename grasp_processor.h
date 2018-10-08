#ifndef GRASP_PROCESSOR_H
#define GRASP_PROCESSOR_H

#include "pcl_includes.h"
#include "poseestimator.h"
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

class GraspProcessor
{
public:
  GraspProcessor(std::string data_dir);
  bool process_grasp(std::string object_name, std::string session_name,
                     bool no_rollpitch, bool symmetric_object,
                     float azim_search_range);

private:
  PointCloudT::Ptr scene_cloud, object_cloud;
  boost::shared_ptr<PoseEstimator> pe;
  bfs::path data_dir;
  bool plane_locked;
  std::string real_object_name;

  bool process_view(bfs::path pc_filename, bfs::path base_dir);
};

#endif // GRASP_PROCESSOR_H
