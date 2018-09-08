#include "grasp_processor.h"

int main(int argc, char **argv) {
    GraspProcessor gp("/home/samarth/deepgrasp_data/data/");
    bool done = gp.process_grasp("binoculars", "full1_use");

    return 0;
}
