#include "voxelizer.h"

using voxelizer::VoxelMeta;

void PromptForVoxel(VoxelMeta& voxel_meta) {
  voxel_meta.SetIndex(1);
  voxel_meta.SetFilled();
}

int main(int argc, char* argv[]) {
  VoxelMeta voxel_meta;
  PromptForVoxel(voxel_meta);
  if (voxel_meta.Index() != 1) {
    return 1;
  }
  if (!voxel_meta.Filled()) {
    return 1;
  }
  return 0;
}
