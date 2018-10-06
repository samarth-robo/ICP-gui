#include "grasp_processor.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  if (argc < 4) {
    cerr << "Usage: ./"
         << argv[0]
         <<" data_dir object_name session_name [no roll/pitch] [symmetric object]"
         << endl;
    return -1;
  }
  GraspProcessor gp(argv[1]);
  bool no_rollpitch(false), symmetric_object(false);
  if (argc >= 5) {
    no_rollpitch = bool(stoi(argv[4]));
    symmetric_object = bool(stoi(argv[5]));
  }
  bool done = gp.process_grasp(argv[2], argv[3], no_rollpitch, symmetric_object);

  return 0;
}
