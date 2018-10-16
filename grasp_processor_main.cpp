#include "grasp_processor.h"
#include <pcl/console/parse.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  if (argc < 4) {
    cerr << "Usage: ./"
         << argv[0]
         <<" data_dir object_name session_name "
           "[--no_rollpitch] [--symmetric] [--only_xy] [--azim_range D.f] "
           "[--plane_from object_name]"
         << endl;
    return -1;
  }
  GraspProcessor gp(argv[1]);
  bool no_rollpitch(pcl::console::find_switch(argc, argv, "--no_rollpitch"));
  bool symmetric_object(pcl::console::find_switch(argc, argv, "--symmetric"));
  bool only_xy(pcl::console::find_switch(argc, argv, "--only_xy"));
  float azim_search_range(360.f);
  pcl::console::parse_argument(argc, argv, "--azim_range", azim_search_range);
  string plane_from;
  pcl::console::parse_argument(argc, argv, "--plane_from", plane_from);
  bool done = gp.process_grasp(argv[2], argv[3], no_rollpitch, symmetric_object,
      only_xy, azim_search_range, plane_from);

  return 0;
}
