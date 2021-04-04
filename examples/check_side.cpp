#include "fcl/shape/geometric_shapes.h"
#include "commons.h"
#include "ray_caster.h"
#include "voxelizer.h"
#include "time.h"

using voxelizer::Voxelizer;
using voxelizer::RayCaster;
using voxelizer::RandFloat;

bool InitVoxleizer(std::unique_ptr<Voxelizer>& vox_ptr) {
  const std::string in_file = "../data/sphere.obj";
  voxelizer::Option option;
  option.SetInFilePath(in_file);
  vox_ptr.reset(new Voxelizer(128, option));
  auto status = vox_ptr->Init();
  if (!status.ok()) {
    std::cout << "init failure: " << status.message() << std::endl;
    return false;
  }
  vox_ptr->VoxelizeSurface(4);
  return true;
}

Vec3f RandomPoint(const Vec3f& lb, const Vec3f& ub) {
  return lb + (ub - lb) * Vec3f(RandFloat(), RandFloat(), RandFloat());
}

int main(int argc, char* argv[]) { 
  srand((unsigned)time(NULL));
  std::unique_ptr<Voxelizer> vox_ptr;
  if (!InitVoxleizer(vox_ptr)) return 1;
  std::vector<Vec3f> vertices = vox_ptr->VerticesVec();
  std::vector<Triangle> triangles = vox_ptr->TrianglesVec();
  std::cout << vertices.size() << ", " << triangles.size() << std::endl;
  RayCaster ray_caster(vertices, triangles);
  if (!ray_caster.Init()) {
    std::cout << "Fail to init ray_caster" << std::endl;
    return 1;
  }
  std::cout << "Space: " << vox_ptr->MeshLowerBound() << ", " << vox_ptr->MeshUpperBound() << std::endl;
  Vec3f source = RandomPoint(vox_ptr->MeshLowerBound(), vox_ptr->MeshUpperBound());
  std::cout << "Source: " << source << std::endl;
  std::cout << "Inside: " << ray_caster.Inside(source) << std::endl;
  std::cout << RandFloat() << ", " << RandFloat() << ", " << RandFloat() << std::endl;
  return 0;
}