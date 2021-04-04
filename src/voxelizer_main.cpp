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
using voxelizer::Option;

ABSL_FLAG(std::vector<std::string>, grid_size, {}, "grid size, the granularity of voxelizer. if only one integer is set, assuming the X,Y,Z are the same.");
ABSL_FLAG(std::vector<std::string>, voxel_size, {}, "voxel size, which determines the size of each voxel");
ABSL_FLAG(int, num_thread, 4, "number of thread to run voxelizer");
ABSL_FLAG(bool, verbose, false, "print debug info");
ABSL_FLAG(std::string, input, "", "input file to be voxelized, file type will be inferred from file suffix");
ABSL_FLAG(std::string, output, "", "output file to store voxelized result");
ABSL_FLAG(std::string, format, "binvox", "output format, can be binvox or rawvox");
ABSL_FLAG(std::string, mode, "solid", "voxelizer mode, surface or solid");
ABSL_FLAG(int, mesh_index, 0, "mesh index to be voxelized");
ABSL_FLAG(std::vector<std::string>, clipping_size, {}, "clipping size (x,y,z) to clip the voxelized result. The clipping size is grid size based. If only one integter is specified, clipping size is initialized as (x,x,x). If not set, the result is not clipped.");
ABSL_FLAG(bool, with_meta, false, "write voxel meta info to .meta file if set to true");
ABSL_FLAG(bool, tight, false, "If true, check whether the center point of each voxel is inside the mesh or outside it, and if it is outside, remove it. Caveat: this option could be time consuming.");

absl::Status PraseOption(Option& option) {
  std::string format = absl::GetFlag(FLAGS_format);
  option.SetFormat(format);

  std::vector<int> clipping_size;
  if (!ToVector3Int(absl::GetFlag(FLAGS_clipping_size), clipping_size)) {
    return absl::InvalidArgumentError("Failed to convert the clipping size to int vector.");
  }
  if (clipping_size.size() == 1) {
    clipping_size.push_back(clipping_size[0]);
    clipping_size.push_back(clipping_size[0]);
  }
  option.SetClippingSize(clipping_size);

  std::string input_file = absl::GetFlag(FLAGS_input);
  if (input_file.empty()) {
    return absl::InvalidArgumentError("Input should be non-empty");
  }
  option.SetInFilePath(input_file);

  std::string output_file = absl::GetFlag(FLAGS_output);
  if (output_file.empty()) {
    return absl::InvalidArgumentError("Output should be non-empty");
  }
  option.SetOutFilePath(output_file);

  int mesh_index = absl::GetFlag(FLAGS_mesh_index);
  option.SetMeshIndex(mesh_index);

  bool verbose = absl::GetFlag(FLAGS_verbose);
  option.SetVerbose(verbose);

  bool tight = absl::GetFlag(FLAGS_tight);
  option.SetTight(tight);

  // tight fit needs with meta.
  bool with_meta = absl::GetFlag(FLAGS_with_meta) || tight;
  option.SetWithMeta(with_meta);

  

  return absl::OkStatus(); 
}

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
  ABSL_INTERNAL_CHECK(voxel_size.size() == 0 || voxel_size.size() == 1 || voxel_size.size() == 3, "voxel_size should be 3 dimensions if specifed");
  if (voxel_size.size() == 1) {
    voxel_size.push_back(voxel_size[0]);
    voxel_size.push_back(voxel_size[0]);
  }
  int num_thread = absl::GetFlag(FLAGS_num_thread);
  
  std::string mode = absl::GetFlag(FLAGS_mode);
  
  ABSL_INTERNAL_CHECK(grid_size.empty() ^ voxel_size.empty(), "if and only if grid_size or voxel_size should be set.");

  Option option;
  auto status = PraseOption(option);
  if (!status.ok()) {
    std::cout << "Failed to parse Option." << std::endl;
    return 1;
  }

  Timer timer;

  timer.Restart();
  Voxelizer* voxelizer;
  if (grid_size.empty()) {
    voxelizer = new Voxelizer(voxel_size, option);
  } else {
    voxelizer = new Voxelizer(grid_size, option);
  }
  status = voxelizer->Init();
  if (!status.ok()) {
    std::cout << "voxelizer fails initialization: " << status.message();
    return 1;
  }
  timer.Stop();
  std::cout << "voxelizer initialization ";
  timer.PrintTimeInMs();
  std::cout << "-------------------------------------------" << std::endl;
  timer.Restart();
  voxelizer->VoxelizeSurface(num_thread);
  timer.Stop();
  std::cout << "surface voxelization ";
  timer.PrintTimeInMs();
  if (mode == "solid") {
    std::cout << "-------------------------------------------" << std::endl;
    timer.Restart();
    voxelizer->VoxelizeSolid(num_thread);
    timer.Stop();
    std::cout << "solid voxelization ";
    timer.PrintTimeInMs();
  }
  std::cout << "-------------------------------------------" << std::endl;
  timer.Restart();
  voxelizer->Write();
  timer.Stop();
  std::cout << "writing file ";
  timer.PrintTimeInMs();

  return 0;
}
