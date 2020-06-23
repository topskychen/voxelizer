/*
 * voxelizerMain.cpp
 *
 *  Created on: 1 Jul, 2014
 *      Author: chenqian
 */
#include <boost/program_options.hpp>

#include "voxelizer.h"

using voxelizer::Timer;
using voxelizer::Voxelizer;

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "grid_size", po::value<int>()->default_value(256),
      "grid size of [1, 1024], the granularity of voxelizer")(
      "num_thread", po::value<int>()->default_value(4),
      "number of thread to run voxelizer")(
      "verbose", po::value<bool>()->default_value(false), "print debug info")(
      "input", po::value<string>(),
      "input file to be voxelized, file type will be inferred from file "
      "suffix")("output", po::value<string>(),
                "output file to store voxelized result")(
      "format", po::value<string>()->default_value("binvox"),
      "output format, can be binvox, rawvox, or cmpvox")(
      "mode", po::value<string>()->default_value("solid"),
      "voxelizer mode, surface or solid")("mesh_index",
                                          po::value<int>()->default_value(0),
                                          "mesh index to be voxelized");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << endl;
    return 1;
  }

  int grid_size = vm["grid_size"].as<int>();

  if (grid_size > 1024) {
    cout << "current only supports not greater than 1024. contact "
            "topskychen@gmail.com if you need more grid_size."
         << endl;
    return 1;
  }

  int num_thread = vm["num_thread"].as<int>();
  int mesh_index = vm["mesh_index"].as<int>();
  string input_file = vm["input"].as<string>();
  string output_file = vm["output"].as<string>();
  string format = vm["format"].as<string>();
  string mode = vm["mode"].as<string>();
  bool verbose = vm["verbose"].as<bool>();

  Timer timer;

  timer.Restart();
  Voxelizer voxelizer(grid_size, input_file, mesh_index, verbose);
  if (!voxelizer.Init()) {
    cout << "voxelizer fails initialization ";
    return 1;
  }
  timer.Stop();
  cout << "voxelizer initialization ";
  timer.PrintTimeInS();
  cout << "-------------------------------------------" << endl;
  timer.Restart();
  voxelizer.VoxelizeSurface(num_thread);
  timer.Stop();
  cout << "surface voxelization ";
  timer.PrintTimeInS();
  if (mode == "solid") {
    cout << "-------------------------------------------" << endl;
    timer.Restart();
    voxelizer.VoxelizeSolid(num_thread);
    timer.Stop();
    cout << "solid voxelization ";
    timer.PrintTimeInS();
  }
  cout << "-------------------------------------------" << endl;
  timer.Restart();
  voxelizer.Write(output_file, format);
  timer.Stop();
  cout << "writing file ";
  timer.PrintTimeInS();

  return 0;
}
