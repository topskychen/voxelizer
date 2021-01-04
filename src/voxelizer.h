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

class Option {
public:
  Option() {
    mesh_index_ = 0;
    verbose_ = false;
    with_meta_=  false;
  }
  std::vector<int> ClippingSize() const {
    return clipping_size_;
  }
  void SetClippingSize(const std::vector<int>& clipping_size) {
     clipping_size_ = clipping_size;
  }
  std::string Format() const {
    return format_;
  }
  void SetFormat(const std::string& format) {
    format_ = format;
  }
  std::string InFilePath() const {
    return in_file_path_;
  }
  void SetInFilePath(const std::string& in_file_path) {
    in_file_path_ = in_file_path;
  }
  std::string OutFilePath() const {
    return out_file_path_;
  }
  void SetOutFilePath(const std::string& out_file_path) {
    out_file_path_ = out_file_path;
  }
  int MeshIndex() const {
    return mesh_index_;
  }
  void SetMeshIndex(const int mesh_index) {
    mesh_index_ = mesh_index;
  }
  bool Verbose() {
    return verbose_;
  }
  void SetVerbose(const bool verbose) {
    verbose_ = verbose;
  }
  bool WithMeta() const {
    return with_meta_;
  }
  void SetWithMeta(const bool with_meta) {
    with_meta_ = with_meta;
  }

private:
  std::string in_file_path_;
  std::string out_file_path_;
  std::string format_;
  std::vector<int> clipping_size_;
  int mesh_index_;
  bool verbose_;
  bool with_meta_;
};

enum VoxelFlagsEnum {
    FILLED            = 0x01 <<  0,        // Set this bit when a voxel cell is not empty. Unset this bit when this voxel is empty.
    INSIDE            = 0x01 <<  1,        // Voxel center is inside the mesh surface , TIGHT
    OUTSIDE           = 0x01 <<  2,        // Voxel center is outside the mesh surface , NOT TIGHT
    SURFACE           = 0x01 <<  3,        // Voxel is generated from surface in the surface voxelizing pass
    SOLID             = 0x01 <<  4,        // Voxel is generated in solid flood filling in the solid voxelizing pass
    CLIPPED           = 0x01 <<  5,        // Voxel is clipped
};

class VoxelMeta {
public:
  VoxelMeta(VoxelIndex index, VoxelFlags flags): index_(index), flags_(flags) {}
  VoxelMeta() {
    Reset();
  }
  void Reset() {
    index_ = 0;
    flags_ = 0;
  }
  VoxelIndex Index() const {
    return index_;
  }
  void SetIndex(const VoxelIndex& index) {
    index_ = index;
  }
  VoxelFlags Flags() const {
    return flags_;
  }
  void SetFlags(const VoxelFlags& flags) {
    flags_ = flags;
  }
  void ClearFlags() {
    flags_ = 0;
  }
  bool Filled() const {
    return (flags_ & VoxelFlagsEnum::FILLED) > 0;
  }
  void SetFilled() {
    flags_ |= VoxelFlagsEnum::FILLED;
  }
  void UnsetFilled() {
    flags_ &= ~VoxelFlagsEnum::FILLED;
  }
  void UpdateFilled() {
    if (!Clipped() && (Solid() || Surface())) {
      SetFilled();
    } else {
      UnsetFilled();
    }
  }
  bool Inside() const {
    return (flags_ & VoxelFlagsEnum::INSIDE) > 0;
  }
  void SetInside() {
    flags_ |= VoxelFlagsEnum::INSIDE;
  }
  void UnsetInside() {
   flags_ &= ~VoxelFlagsEnum::INSIDE; 
  }
  bool Outside() const {
    return (flags_ & VoxelFlagsEnum::OUTSIDE) > 0;
  }
  void SetOutSide() {
    flags_ |= VoxelFlagsEnum::OUTSIDE;
  }
  void UnsetOutSide() {
    flags_ &= ~VoxelFlagsEnum::OUTSIDE;
  }
  bool Surface() const {
    return (flags_ & VoxelFlagsEnum::SURFACE) > 0;
  }
  void SetSurface() {
    flags_ |= VoxelFlagsEnum::SURFACE;
  }
  void UnsetSurface() {
    flags_ &= ~VoxelFlagsEnum::SURFACE;
  }
  bool Solid() const {
    return (flags_ & VoxelFlagsEnum::SOLID) > 0;
  }
  void SetSolid() {
    flags_ |= VoxelFlagsEnum::SOLID; 
  }
  void UnsetSolid() {
    flags_ &= ~VoxelFlagsEnum::SOLID; 
  }
  bool Clipped() const {
    return (flags_ & VoxelFlagsEnum::CLIPPED) > 0;
  }
  void SetClipped() {
    flags_ |= VoxelFlagsEnum::CLIPPED;
  }
  void UnsetClipped() {
    flags_ &= ~VoxelFlagsEnum::CLIPPED;
  }

private:
  VoxelIndex index_;
  VoxelFlags flags_;
};

class Voxelizer {
  Option option_;
  boost::shared_ptr<VoxelMeta> voxel_metas_;
  bool is_init_;

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
  void GetOutputBound(Vec3f& output_lb, Vec3f& output_ub);
  void UpdateSurfaceMeta();
  void UpdateSolidMeta();
  void InitVoxelMeta();

 public:
  VoxelIndex TotalVoxelSize();
  VoxelIndex TotalVoxelCompressedSize();
  Vec3f HalfUnit();
  Vec3f Unit();
  AVISP Voxels();
  Vec3f GetVoxel(const Vec3f& loc);
  Vec3f GetLoc(const Vec3f& voxel);
  Vec3f MeshLowerBound();
  Vec3f MeshUpperBound();
  Vec3f LowerBound();
  Vec3f UpperBound();
  int VerticesSize();
  int FacesSize();
  V3SP Vertices();
  std::vector<Vec3f> VerticesVec();
  V3SP Faces();
  std::vector<Triangle> TrianglesVec();
  void VoxelizeSurface(int num_thread = 1);
  void VoxelizeSolid(int num_thread = 1);
  void Write();
  void WriteBinvox();
  void WriteRawvox();
  void WriteMeta();
  absl::Status Init();
  void SetOption(const Option& option) {
    option_ = option;
  }
  Voxelizer(int grid_size, const Option& option)
      : option_(option) {
        grid_size_.push_back(grid_size);
        grid_size_.push_back(grid_size);
        grid_size_.push_back(grid_size);
      }
  Voxelizer(float voxel_size, const Option& option)
      : option_(option) {
        voxel_size_.push_back(voxel_size);
        voxel_size_.push_back(voxel_size);
        voxel_size_.push_back(voxel_size);
      }
  Voxelizer(const std::vector<int>& grid_size, const Option& option)
      : grid_size_(grid_size), option_(option) {}
  Voxelizer(const std::vector<float>& voxel_size, const Option& option)
      : voxel_size_(voxel_size), option_(option) {}
  virtual ~Voxelizer();
};

}  // namespace voxelizer

#endif /* VOXELIZER_H_ */
