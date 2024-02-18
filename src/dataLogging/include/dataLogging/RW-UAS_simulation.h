// RW-UAS_simulation.h : Include file for standard system include files,
// or project specific include files.

#pragma once

// C++ standard library headers
#include <iostream>
#include <list>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <string>
#include <chrono>
#include <cstdio>
// in order to run the polyscope visual and haply inverse kinematics in seperate threads
#include <thread>
#include <mutex>
#include <fstream>
#include <filesystem>
namespace std
{
#include <cstdlib>
};

// Eigen stuff
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// polyscope headers and its deps
#include "../deps/polyscope/include/polyscope/polyscope.h"
//#include <polyscope/polyscope.h>    //!
#include "../deps/polyscope/include/polyscope/messages.h"
#include "../deps/polyscope/include/polyscope/point_cloud.h"     //!!!
#include "../deps/polyscope/include/polyscope/surface_mesh.h"
#include "../deps/polyscope/deps/args/args/args.hxx"

// igl headers and its deps
#include <igl/readOBJ.h>

// json header and its deps
#include "../deps/json/single_include/nlohmann/json.hpp"   //!

//haply header and its deps
#include "../deps/HardwareAPI/include/HardwareAPI.h"

// include all the header files for the internal libraries
#include "../include/RW-UAS_simulation/BRIM.h"
#include "../include/RW-UAS_simulation/BubbleMethod.h"
#include "../include/RW-UAS_simulation/PRIM.h"
#include "../include/RW-UAS_simulation/PX4Comms.h"
#include "../include/RW-UAS_simulation/Q_helpers.h"
#include "../include/RW-UAS_simulation/Quadcopter.h"
#include "../include/RW-UAS_simulation/RB_helpers.h"
#include "../include/RW-UAS_simulation/RBsystem.h"