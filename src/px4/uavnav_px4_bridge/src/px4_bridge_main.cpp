// The px4_bridge executable. Everything is in uavnav::px4bridge_node::run (src/px4_bridge_node.cpp), which the
// in-process tests do not use; this file names no ROS type.
#include "uavnav/px4bridge_node/px4_bridge_node.hpp"

int main(int argc, char** argv) { return uavnav::px4bridge_node::run(argc, argv); }
