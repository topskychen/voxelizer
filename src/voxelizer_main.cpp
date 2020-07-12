/*
 * voxelizerMain.cpp
 *
 *  Created on: 1 Jul, 2014
 *      Author: chenqian
 */
#include <vector>
#include <string>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/base/internal/raw_logging.h"

#include "voxelizer.h"

using voxelizer::Timer;
using voxelizer::Voxelizer;
using voxelizer::ToVector3Int;
using voxelizer::ToVector3Float;

ABSL_FLAG(std::vector<std::string>, grid_size, {}, "grid size, the granularity of voxelizer. if only one integer is set, assuming the X,Y,Z are the same.");
ABSL_FLAG(std::vector<std::string>, voxel_size, {}, "voxel size, which determines the size of each voxel");
ABSL_FLAG(int, num_thread, 4, "number of thread to run voxelizer");
ABSL_FLAG(bool, verbose, false, "print debug info");
ABSL_FLAG(std::string, input, "", "input file to be voxelized, file type will be inferred from file suffix");
ABSL_FLAG(std::string, output, "", "output file to store voxelized result");
ABSL_FLAG(std::string, format, "binvox", "output format, can be binvox, rawvox, or cmpvox");
ABSL_FLAG(std::string, mode, "solid", "voxelizer mode, surface or solid");
ABSL_FLAG(int, mesh_index, 0, "mesh index to be voxelized");

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);

  std::vector<int> grid_size; 
  ABSL_INTERNAL_CHECK(ToVector3Int(absl::GetFlag(FLAGS_grid_size), grid_size), "failed to parse grid_size");
  ABSL_INTERNAL_CHECK(grid_size.size() == 0 || grid_size.size() == 1 || grid_size.size() == 3, "grid_size should be 1 or 3 dimensions if specifed");
  if (grid_size.size() == 1) {
    grid_size.push_back(grid_size[0]);
    grid_size.push_back(grid_size[0]);
  }
  std::vector<float> voxel_size;
  ABSL_INTERNAL_CHECK(ToVector3Float(absl::GetFlag(FLAGS_voxel_size), voxel_size), "failed to parse voxel_size");
  ABSL_INTERNAL_CHECK(voxel_size.size() == 0 || voxel_size.size() == 3, "voxel_size should be 3 dimensions if specifed");

  int num_thread = absl::GetFlag(FLAGS_num_thread);
  int mesh_index = absl::GetFlag(FLAGS_mesh_index);
  std::string input_file = absl::GetFlag(FLAGS_input);
  std::string output_file = absl::GetFlag(FLAGS_output);
  std::string format = absl::GetFlag(FLAGS_format);
  std::string mode = absl::GetFlag(FLAGS_mode);
  bool verbose = absl::GetFlag(FLAGS_verbose);

  ABSL_INTERNAL_CHECK(!input_file.empty(), "input should be non-empty");
  ABSL_INTERNAL_CHECK(!output_file.empty(), "output should be non-empty");
  ABSL_INTERNAL_CHECK(grid_size.empty() ^ voxel_size.empty(), "if and only if grid_size or voxel_size should be set.");

  Timer timer;

  timer.Restart();
  Voxelizer voxelizer(grid_size, input_file, mesh_index, verbose);
  if (!voxelizer.Init()) {
    std::cout << "voxelizer fails initialization ";
    return 1;
  }
  timer.Stop();
  std::cout << "voxelizer initialization ";
  timer.PrintTimeInMs();
  std::cout << "-------------------------------------------" << std::endl;
  timer.Restart();
  voxelizer.VoxelizeSurface(num_thread);
  timer.Stop();
  std::cout << "surface voxelization ";
  timer.PrintTimeInMs();
  if (mode == "solid") {
    std::cout << "-------------------------------------------" << std::endl;
    timer.Restart();
    voxelizer.VoxelizeSolid(num_thread);
    timer.Stop();
    std::cout << "solid voxelization ";
    timer.PrintTimeInMs();
  }
  std::cout << "-------------------------------------------" << std::endl;
  timer.Restart();
  voxelizer.Write(output_file, format);
  timer.Stop();
  std::cout << "writing file ";
  timer.PrintTimeInMs();

  return 0;
}
