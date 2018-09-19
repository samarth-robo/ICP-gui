#include "grasp_processor.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  if (argc < 4) {
    cerr << "Usage: ./"
         << argv[0]
         <<" data_dir object_name session_name [restrict rotation]" << endl;
    return -1;
  }
  GraspProcessor gp(argv[1]);
  bool restrict_rotation(false);
  if (argc == 5) restrict_rotation = bool(stoi(argv[4]));
  bool done = gp.process_grasp(argv[2], argv[3], restrict_rotation);

  return 0;
}
