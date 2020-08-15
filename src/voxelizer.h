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

#include "absl/status/status.h"
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
#define INDEX(x, y, z) x * size_yz_ + y * size_z_ + z

class OutputOption {
public:
  std::vector<int> GetClippingSize() const;
  void SetClippingSize(const std::vector<int>& clipping_size);
  std::string GetFormat() const;
  void SetFormat(const std::string& format);
  std::string GetFilePath() const;
  void SetFilePath(const std::string& file_path);

private:
  std::string file_path_;
  std::string format_;
  std::vector<int> clipping_size_;
};

class Voxelizer {
  bool is_init_;
  bool verbose_;
  int mesh_index_;
  std::string p_file_;

  V3UP mesh_lb_, mesh_ub_;          // location
  V3UP mesh_vox_lb_, mesh_vox_ub_;  // voxels of location

  float min_lb_, max_ub_;
  V3UP lb_, ub_, bound_;  // lowerBound and upperBound of the whole space
  V3UP unit_, half_unit_;        // full or half size of the unit

  int num_meshes_;

  V3SP faces_;
  int num_faces_;

  V3SP vertices_;
  int num_vertices_;

  AVISP voxels_buffer_;
  AVISP voxels_;

  VoxelIndex size2_, size_x_, size_y_, size_z_, size_xy_, size_xz_, size_yz_, compressed_total_size_;  // size_2 = size*size
  V3UP size_, scale_;

  std::vector<int> grid_size_;
  std::vector<float> voxel_size_;

  absl::Status LoadFromMesh(const aiMesh* mesh);
  inline void RunSolidTask(size_t num_thread = 1);
  inline void RunSolidTask2(size_t num_thread = 1);
  inline void RunSurfaceTask(const int tri_id);
  inline void FillYZ(const int x);
  inline void FillXZ(const int y);
  inline void FillXY(const int z);
  inline void FillYZ2(const int x);
  inline void FillXZ2(const int y);
  inline void FillXY2(const int z);
  inline bool InRange(const Vec3f& vc, const Vec3f& lb, const Vec3f& ub);
  inline bool InRange(const int x, const int y, const int z, const int lx,
                      const int ly, const int lz, const int ux,
                      const int uy, const int uz);
  inline bool InRange(const int x, const int y, const int z,
                               const Vec3f& lb, const Vec3f& ub);
  inline TriangleP GetTri(const int tri_id);
  inline Vec3f ConvIndexToVoxel(const VoxelIndex coord);
  void ConvIndexToVoxel(const VoxelIndex coord, Vec3f& voxel);
  inline VoxelIndex ConvVoxelToIndex(const Vec3f& voxel);
  inline VoxelIndex BfsSurface(const TriangleP& tri, const Vec3f& lb, const Vec3f& ub);
  void RandomPermutation(const V3SP& data, int num);
  void BfsSolid(const VoxelIndex voxel_id);
  void GetOutputBound(const OutputOption& output_option, Vec3f& output_lb, Vec3f& output_ub);

 public:
  VoxelIndex GetTotalSize();
  Vec3f GetHalfUnit();
  Vec3f GetUnit();
  AVISP GetVoxels();
  Vec3f GetVoxel(const Vec3f& loc);
  Vec3f GetLoc(const Vec3f& voxel);
  Vec3f GetMeshLowerBound();
  Vec3f GetMeshUpperBound();
  Vec3f GetLowerBound();
  Vec3f GetUpperBound();
  int GetVerticesSize();
  int GetFacesSize();
  V3SP GetVertices();
  V3SP GetFaces();
  void VoxelizeSurface(int num_thread = 1);
  void VoxelizeSolid(int num_thread = 1);
  void Write(const OutputOption& output_option);
  void WriteBinvox(const OutputOption& output_option);
  void WriteRawvox(const OutputOption& output_option);
  absl::Status Init();
  Voxelizer(int grid_size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {
        grid_size_.push_back(grid_size);
        grid_size_.push_back(grid_size);
        grid_size_.push_back(grid_size);
      }
  Voxelizer(float voxel_size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {
        voxel_size_.push_back(voxel_size);
        voxel_size_.push_back(voxel_size);
        voxel_size_.push_back(voxel_size);
      }
  Voxelizer(const std::vector<int>& grid_size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : grid_size_(grid_size), p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {}
  Voxelizer(const std::vector<float>& voxel_size, const std::string& p_file, int mesh_index=0, bool verbose=false)
      : voxel_size_(voxel_size), p_file_(p_file), mesh_index_(mesh_index), verbose_(verbose) {}
  virtual ~Voxelizer();
};

}  // namespace voxelizer

#endif /* VOXELIZER_H_ */
