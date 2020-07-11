/*
 * voxelizerMain.cpp
 *
 *  Created on: 1 Jul, 2014
 *      Author: chenqian
 */
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/base/internal/raw_logging.h"

#include "voxelizer.h"

using voxelizer::Timer;
using voxelizer::Voxelizer;

ABSL_FLAG(int, grid_size, 256, "grid size of [1, 1024], the granularity of voxelizer");
ABSL_FLAG(int, num_thread, 4, "number of thread to run voxelizer");
ABSL_FLAG(bool, verbose, false, "print debug info");
ABSL_FLAG(std::string, input, "", "input file to be voxelized, file type will be inferred from file suffix");
ABSL_FLAG(std::string, output, "", "output file to store voxelized result");
ABSL_FLAG(std::string, format, "binvox", "output format, can be binvox, rawvox, or cmpvox");
ABSL_FLAG(std::string, mode, "solid", "voxelizer mode, surface or solid");
ABSL_FLAG(int, mesh_index, 0, "mesh index to be voxelized");

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);

  int grid_size = absl::GetFlag(FLAGS_grid_size);    
  int num_thread = absl::GetFlag(FLAGS_num_thread);
  int mesh_index = absl::GetFlag(FLAGS_mesh_index);
  std::string input_file = absl::GetFlag(FLAGS_input);
  std::string output_file = absl::GetFlag(FLAGS_output);
  std::string format = absl::GetFlag(FLAGS_format);
  std::string mode = absl::GetFlag(FLAGS_mode);
  bool verbose = absl::GetFlag(FLAGS_verbose);

  ABSL_INTERNAL_CHECK(!input_file.empty(), "input should be non-empty");
  ABSL_INTERNAL_CHECK(!output_file.empty(), "output should be non-empty");
  ABSL_INTERNAL_CHECK(grid_size <= 1024, "currently this voxelizer only supports not greater than 1024. contact topskychen@gmail.com if you need more grid_size.");

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
