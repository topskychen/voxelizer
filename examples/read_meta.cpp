#include <fstream>
#include <iostream>
#include <string>
#include "commons.h"
#include "voxelizer.h"

using voxelizer::VoxelIndex;
using voxelizer::VoxelFlags;
using voxelizer::VoxelMeta;

int main(int argc, char* argv[]) {
  std::ifstream* input = new std::ifstream(argv[1], std::ios::in | std::ios::binary);
  std::string format;
  VoxelIndex size_x, size_y, size_z;
  getline(*input, format);
  std::cout << format << std::endl;
  *input >> size_x >> size_y >> size_z;
  std::cout << size_x << ", " << size_y << ", " << size_z << std::endl;
  VoxelIndex total_size = size_x * size_y * size_z;
  int filled = 0;
  for (VoxelIndex voxel_index = 0; voxel_index < total_size; ++voxel_index) {
    VoxelIndex read_index;
    VoxelFlags read_flag;
    *input >> read_index >> read_flag;
    VoxelMeta voxel_meta(read_index, read_flag);
    if (voxel_meta.Filled()) ++filled;
  }
  std::cout << "Filled: " << filled << std::endl;
  input->close();
}
