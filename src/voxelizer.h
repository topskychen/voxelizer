/*
 * Voxelizer.h
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#ifndef VOXELIZER_H_
#define VOXELIZER_H_

#include <assimp/postprocess.h>  // Post processing flags
#include <assimp/scene.h>        // Output data structure

#include <assimp/Importer.hpp>  // C++ importer interface
#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <queue>

#include "commons.h"
#include "thread_pool.h"
#include "timer.h"

namespace voxelizer {

const int kPrimitiveTriangleType = 0x4;
const int kTriangleNumIndices = 3;
const Vec3f kEpsBox(0.0001, 0.0001, 0.0001); // epsilon box
const int kThresholdBfsSurface = 10;

#define GETBIT(x, i) ((x >> (i % kBatchSize)) & 1)
#define SETBIT(voxels, voxel_index) (voxels.get())[voxel_index / kBatchSize] |= (static_cast<VoxelIndex>(1) << (voxel_index % kBatchSize))

class Voxelizer {
  bool is_init_;
  bool verbose_;
  int mesh_index_;
  std::string p_file_;

  V3SP mesh_lb_, mesh_ub_;          // location
  V3SP mesh_vox_lb_, mesh_vox_ub_;  // voxels of location

  float min_lb_, max_ub_;
  V3SP lb_, ub_, bound_;  // lowerBound and upperBound of the whole space
  V3SP half_unit_;        // half size of the unit

  V3SP faces_;
  int num_faces_;

  V3SP vertices_;
  int num_vertices_;

  AVISP voxels_buffer_;
  AVISP voxels_;

  VoxelIndex size_, size2_, compressed_total_size_;  // size_2 = size*size

  std::vector<int> grid_size_;
  std::vector<float> voxel_size_;

  inline void LoadFromMesh(const aiMesh* mesh);
  inline void RunSolidTask(size_t num_thread = 1);
  inline void RunSolidTask2(size_t num_thread = 1);
  inline void RunSurfaceTask(const int tri_id);
  inline void FillYZ(const int x);
  inline void FillXZ(const int y);
  inline void FillXY(const int z);
  inline void FillYZ2(const int x);
  inline void FillXZ2(const int y);
  inline void FillXY2(const int z);
  inline bool InRange(const Vec3f& vc, const V3SP& lb, const V3SP& ub);
  inline bool InRange(const int& x, const int& y, const int& z, const int& lx,
                      const int& ly, const int& lz, const int& ux,
                      const int& uy, const int& uz);

  inline TriSP GetTri(const int tri_id);
  inline V3SP ConvIndexToVoxel(const VoxelIndex coord);
  inline VoxelIndex ConvVoxelToIndex(const V3SP& voxel);
  inline VoxelIndex ConvVoxelToIndex(const Vec3f& voxel);
  inline VoxelIndex BfsSurface(const TriSP& tri, const V3SP& lb, const V3SP& ub);
  inline void RandomPermutation(const V3SP& data, int num);
  inline void BfsSolid(const VoxelIndex voxel_id);

 public:
  VoxelIndex GetTotalSize();
  V3SP GetHalfUnit();
  AVISP GetVoxels();
  V3SP GetVoxel(const Vec3f& loc);
  V3SP GetVoxel(const V3SP& loc);
  V3SP GetLoc(const V3SP& voxel);
  V3SP GetLoc(const Vec3f& voxel);
  V3SP GetMeshLowerBound();
  V3SP GetMeshUpperBound();
  V3SP GetLowerBound();
  V3SP GetUpperBound();
  int GetVerticesSize();
  int GetFacesSize();
  V3SP GetVertices();
  V3SP GetFaces();
  void VoxelizeSurface(int num_thread = 1);
  void VoxelizeSolid(int num_thread = 1);
  void Write(const std::string& p_file, const std::string& format);
  void WriteBinvox(const std::string& p_file);
  void WriteRawvox(const std::string& p_file);
  void WriteCmpvox(const std::string& p_file);
  bool Init();
  Voxelizer(int size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : size_(size), p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {
        grid_size_.push_back(size);
        grid_size_.push_back(size);
        grid_size_.push_back(size);
        size2_ = size_ * size_;
        compressed_total_size_ = size_ * size_ * size_ / kBatchSize;
      }
  Voxelizer(const std::vector<int>& grid_size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : grid_size_(grid_size), p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {
        size_ = grid_size_[0];
        size2_ = size_ * size_;
        compressed_total_size_ = size_ * size_ * size_ / kBatchSize;
      }
  Voxelizer(const std::vector<float>& voxel_size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : voxel_size_(voxel_size), p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {
        size2_ = size_ * size_;
        compressed_total_size_ = size_ * size_ * size_ / kBatchSize;
      }
  virtual ~Voxelizer();
};

}  // namespace voxelizer

#endif /* VOXELIZER_H_ */
