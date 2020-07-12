/*
 * Voxelizer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "absl/strings/str_format.h"
#include "voxelizer.h"

namespace voxelizer {

absl::Status Voxelizer::Init() {
  if (verbose_) std::cout << "voxelizer init... " << std::endl;

  if (!(grid_size_.empty() ^ voxel_size_.empty())) {
    return absl::InvalidArgumentError("only one of grid_size and voxel_size can be specifed.");
  }

  is_init_ = false;
  const aiScene* scene;
  try {
    /*
     * Load scene
     * */
    Assimp::Importer importer;
    scene = importer.ReadFile(p_file_, aiProcessPreset_TargetRealtime_Quality |
                                           aiProcess_OptimizeGraph |
                                           aiProcess_OptimizeMeshes);
    if (!scene) {
      return absl::AbortedError("Scene fails to be loaded!");
    }
    if (verbose_) std::cout << "mesh number: " << scene->mNumMeshes << std::endl;
    if (scene->mNumMeshes == 0) {
      return absl::AbortedError("0 mesh in the scene!");
    }
    if (scene->mNumMeshes <= mesh_index_) {
      return absl::AbortedError("Required mesh index out of range!");
    }

    // TODO(topskychen@gmail.com): consider all the meshes when mesh_index_ is
    // -1.
    aiMesh* mesh = scene->mMeshes[mesh_index_];

    /**
     * Store info.
     */
    num_vertices_ = mesh->mNumVertices;
    num_faces_ = mesh->mNumFaces;
    if (verbose_) std::cout << "faces : " << num_faces_ << std::endl;
    if (verbose_) std::cout << "vertices : " << num_vertices_ << std::endl;
    
    /**
    * Load meshes.
    */
    auto status = LoadFromMesh(mesh);
    if (!status.ok()) return status;

    is_init_ = true;
  } catch (std::exception& e) {
    return absl::AbortedError(e.what());
  }
  if (verbose_) std::cout << "done." << std::endl;

  return absl::OkStatus();
}

/**
 * Given voxel (int x, int y, int z), return loc (float x, float y, float z)
 */
Vec3f Voxelizer::GetLoc(const Vec3f& voxel) {
  return *lb_ + (*scale_) * (voxel);
}

/**
 * Given loc (float x, float y, float z), return voxel (int x, int y, int z)
 */
Vec3f Voxelizer::GetVoxel(const Vec3f& loc) {
  Vec3f tmp = (loc - (*lb_)) * (*size_) / (*bound_);
  return Vec3f(static_cast<int>(tmp[0]), static_cast<int>(tmp[1]), static_cast<int>(tmp[2]));
}

/**
 *Get the collision object form triangle(face) id;
 */
TriangleP Voxelizer::GetTri(const int tri_id) {
  const Vec3f& v_ids = faces_.get()[tri_id];
  return TriangleP(vertices_.get()[(int)v_ids[0]],
                          vertices_.get()[(int)v_ids[1]],
                          vertices_.get()[(int)v_ids[2]]);
}

/**
 * Load info from mesh.
 */
absl::Status Voxelizer::LoadFromMesh(const aiMesh* mesh) {
  // load vertices and update mesh bounds
  vertices_.reset(new Vec3f[num_vertices_], ArrayDeleter<Vec3f>());
  Vec3f tmp;
  for (size_t i = 0; i < num_vertices_; ++i) {
    vertices_.get()[i] =
        Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
    if (i == 0) {
      mesh_lb_.reset(new Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
                               mesh->mVertices[i].z));
      mesh_ub_.reset(new Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
                               mesh->mVertices[i].z));
    } else {
      mesh_lb_->ubound(vertices_.get()[i]);  // bug1
      mesh_ub_->lbound(vertices_.get()[i]);  // bug2
    }
  }

  // Add an epsilon box to bound box the whole mesh spce.
  mesh_lb_.reset(new Vec3f((*mesh_lb_) - kEpsBox));
  mesh_ub_.reset(new Vec3f((*mesh_ub_) + kEpsBox));

  /**
   * Calculate a bounding box according to the mesh bounds.
   */
  min_lb_ = (*mesh_lb_)[0];
  min_lb_ = std::min(min_lb_, (float)(*mesh_lb_)[1]);
  min_lb_ = std::min(min_lb_, (float)(*mesh_lb_)[2]);
  max_ub_ = (*mesh_ub_)[0];
  max_ub_ = std::max(max_ub_, (float)(*mesh_ub_)[1]);
  max_ub_ = std::max(max_ub_, (float)(*mesh_ub_)[2]);
  lb_ = absl::make_unique<Vec3f>(min_lb_, min_lb_, min_lb_);
  ub_ = absl::make_unique<Vec3f>(max_ub_, max_ub_, max_ub_);
  bound_ = absl::make_unique<Vec3f>(*ub_ - *lb_);

  /**
  * Store face info.
  */
  faces_.reset(new Vec3f[num_faces_], ArrayDeleter<Vec3f>());
  if (mesh->mPrimitiveTypes != kPrimitiveTriangleType) {
    return absl::InvalidArgumentError(absl::StrFormat("mesh face primitive type expects: %d indices but received: %d", kPrimitiveTriangleType, mesh->mPrimitiveTypes));
  }
  for (size_t i = 0; i < num_faces_; ++i) {
    if (mesh->mFaces[i].mNumIndices != kTriangleNumIndices) {
      return absl::InvalidArgumentError(absl::StrFormat("triangle face expects: %d indices but received %d", kTriangleNumIndices, mesh->mFaces[i].mNumIndices));
    }
    faces_.get()[i] =
        Vec3f(mesh->mFaces[i].mIndices[0], mesh->mFaces[i].mIndices[1],
              mesh->mFaces[i].mIndices[2]);
  }
  RandomPermutation(faces_, num_faces_);

  /**
  * Calculate the grid size and voxel size. 
  */
  if (grid_size_.size() > 0) {
    if (grid_size_.size() != 3) {
      return absl::InvalidArgumentError(absl::StrFormat("grid_size dim error: %d", grid_size_.size()));
    }

    size_x_ = grid_size_[0];
    size_y_ = grid_size_[1];
    size_z_ = grid_size_[2];

    size_ = absl::make_unique<Vec3f>(size_x_, size_y_, size_z_);

    unit_ = absl::make_unique<Vec3f>((*bound_)/(*size_));
    half_unit_ = absl::make_unique<Vec3f>((*unit_)/2);
  } else {
    if (voxel_size_.size() != 3) {
      return absl::InvalidArgumentError(absl::StrFormat("voxel_size dim error: %d", voxel_size_.size()));
    }

    float voxel_x = voxel_size_[0];
    float voxel_y = voxel_size_[1];
    float voxel_z = voxel_size_[2];

    unit_ = absl::make_unique<Vec3f>(voxel_x, voxel_y, voxel_z);
    half_unit_ = absl::make_unique<Vec3f>((*unit_)/2);

    Vec3f size = (*bound_)/(*unit_);
    
    size_x_ = static_cast<VoxelIndex>(size[0]);
    size_y_ = static_cast<VoxelIndex>(size[1]);
    size_z_ = static_cast<VoxelIndex>(size[2]);

    size_ = absl::make_unique<Vec3f>(size_x_, size_y_, size_z_);
  }

  if (verbose_) {
    std::cout << absl::StrFormat("voxel size: %f, %f, %f", (*unit_)[0], (*unit_)[1], (*unit_)[2]) << std::endl;
    std::cout << absl::StrFormat("grid size: %d, %d, %d", size_x_, size_y_, size_z_) << std::endl; 
  }

  // The real voxel bounding box should extract the epsilon box.
  mesh_vox_lb_ = absl::make_unique<Vec3f>(GetVoxel(*mesh_lb_ + kEpsBox));
  mesh_vox_ub_ = absl::make_unique<Vec3f>(GetVoxel(*mesh_ub_ - kEpsBox));

  if (verbose_) {
    std::cout << "space: " << *lb_ << ", " << *ub_ << std::endl;
    std::cout << "mesh bound: " << *mesh_lb_ << ", " << *mesh_ub_ << std::endl;
    std::cout << "voxel bound: " << *mesh_vox_lb_ << ", " << *mesh_vox_ub_ << std::endl;
  }

  size_xy_ = size_x_*size_y_;
  size_xz_ = size_x_*size_z_;
  size_yz_ = size_y_*size_z_;
  scale_ = absl::make_unique<Vec3f>((*bound_) / (*size_));
  compressed_total_size_ = size_x_*size_y_*size_z_/kBatchSize;

  /**
  * Reset voxels.
  */
  voxels_.reset(new AVI[compressed_total_size_], ArrayDeleter<AVI>());
  voxels_buffer_.reset(new AVI[compressed_total_size_], ArrayDeleter<AVI>());
  memset(voxels_.get(), 0, compressed_total_size_ * sizeof(VoxelIndex));
  memset(voxels_buffer_.get(), 0, compressed_total_size_ * sizeof(VoxelIndex));

  return absl::OkStatus();
}

/**
 * voxelize the surface.
 */
void Voxelizer::VoxelizeSurface(const int num_thread) {
  if (!is_init_) {
    return;
  }
  if (verbose_) std::cout << "surface voxelizing... " << std::endl;
  ThreadPool tp(num_thread);
  for (int i = 0; i < num_faces_; ++i) {
    tp.Run(boost::bind(&Voxelizer::RunSurfaceTask, this, i));
  }
  tp.Stop();
  if (verbose_) std::cout << "done." << std::endl;
}

/**
 * Details of surface task.
 */
inline void Voxelizer::RunSurfaceTask(const int tri_id) {
  TriangleP tri = GetTri(tri_id);
  tri.computeLocalAABB();
  const Vec3f lb = GetVoxel(tri.aabb_local.min_);
  const Vec3f ub = GetVoxel(tri.aabb_local.max_);
  
  const int lx = lb[0], ux = ub[0], ly = lb[1], uy = ub[1], lz = lb[2],
      uz = ub[2];
  /**
   * when the estimated voxels are too large, optimize with bfs.
   */
  VoxelIndex count = 0;
  int esti = std::min(ux - lx, std::min(uy - ly, uz - lz));
  if (esti < kThresholdBfsSurface) {
    VoxelIndex voxel_index, tmp;
    Vec3f vxlBox(0, 0, 0);
    for (int x = lx, y, z; x <= ux; ++x) {
      for (y = ly; y <= uy; ++y) {
        for (z = lz; z <= uz; ++z) {
          voxel_index = INDEX(x, y, z);
          tmp = (voxels_.get())[voxel_index / kBatchSize].load();
          if (GETBIT(tmp, voxel_index)) continue;
          vxlBox.setValue(x, y, z);
          if (Collide(*half_unit_, GetLoc(vxlBox), tri)) {
            SETBIT(voxels_, voxel_index);
            count++;
          }
        }
      }
    }
  } else {
    count = BfsSurface(tri, lb, ub);
  }
}

inline VoxelIndex Voxelizer::BfsSurface(const TriangleP& tri, const Vec3f& lb,
                                 const Vec3f& ub) {
  std::queue<VoxelIndex> q;
  HashSet set;
  VoxelIndex start = ConvVoxelToIndex(GetVoxel(tri.a)), top_voxel_index, tmp,
               new_voxel_index;
  q.push(start);
  set.insert(start);
  Vec3f top_voxel;
  VoxelIndex count = 0;
  while (!q.empty()) {
    count++;
    top_voxel_index = q.front();
    q.pop();
    tmp = (voxels_.get())[top_voxel_index / kBatchSize].load();
    ConvIndexToVoxel(top_voxel_index, top_voxel);
    if (GETBIT(tmp, top_voxel_index) ||
        Collide(*half_unit_, GetLoc(top_voxel), tri)) {
      if (!GETBIT(tmp, top_voxel_index)) {
        SETBIT(voxels_, top_voxel_index);
      }
      for (int i = 0; i < 6; ++i) {
        Vec3f newVoxel = top_voxel + D_6[i];
        if (!InRange(newVoxel, lb, ub)) continue;
        new_voxel_index = ConvVoxelToIndex(newVoxel);
        if (set.find(new_voxel_index) == set.end()) {
          set.insert(new_voxel_index);
          q.push(new_voxel_index);
        }
      }
    }
  }
  return count;
}

void Voxelizer::VoxelizeSolid(int num_thread) {
  if (!is_init_) {
    return;
  }
  if (verbose_) std::cout << "solid voxelizing... " << std::endl;
  if (verbose_) std::cout << "round 1..." << std::endl;
  RunSolidTask(num_thread);
  if (verbose_) std::cout << "round 2..." << std::endl;
  RunSolidTask2(num_thread);
  for (VoxelIndex i = 0; i < compressed_total_size_; ++i)
    voxels_.get()[i] = voxels_buffer_.get()[i] ^ (~0);
  if (verbose_) std::cout << "done." << std::endl;
}

inline void Voxelizer::BfsSolid(const VoxelIndex start_index) {
  VoxelIndex voxel_index = start_index,
               tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                     (voxels_buffer_.get())[voxel_index / kBatchSize].load();
  if (GETBIT(tmp, voxel_index)) return;
  std::queue<VoxelIndex> q;
  q.push(voxel_index);
  Vec3f top_voxel;
  while (!q.empty()) {
    voxel_index = q.front();
    tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
          (voxels_buffer_.get())[voxel_index / kBatchSize].load();
    q.pop();
    ConvIndexToVoxel(voxel_index, top_voxel);
    if (!GETBIT(tmp, voxel_index)) {
      SETBIT(voxels_buffer_, voxel_index);
      for (int i = 0; i < 6; i++) {
        Vec3f newVoxel = top_voxel + D_6[i];
        if (!InRange(newVoxel, *mesh_vox_lb_, *mesh_vox_ub_)) continue;
        voxel_index = ConvVoxelToIndex(newVoxel);
        tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
              (voxels_buffer_.get())[voxel_index / kBatchSize].load();
        if (!GETBIT(tmp, voxel_index)) q.push(voxel_index);
      }
    }
  }
}

inline bool Voxelizer::InRange(const Vec3f& vc, const Vec3f& lb,
                               const Vec3f& ub) {
  return vc[0] >= lb[0] && vc[0] <= ub[0] && vc[1] >= lb[1] &&
         vc[1] <= ub[1] && vc[2] >= lb[2] && vc[2] <= ub[2];
}

inline bool Voxelizer::InRange(const int x, const int y, const int z,
                               const int lx, const int ly, const int lz,
                               const int ux, const int uy, const int uz) {
  return x >= lx && x <= ux && y >= ly && y <= uy && z >= lz && z <= uz;
}

inline Vec3f Voxelizer::ConvIndexToVoxel(const VoxelIndex coord) {
  return Vec3f(coord / size_yz_, (coord / size_z_) % size_y_, coord % size_z_);
}

void Voxelizer::ConvIndexToVoxel(const VoxelIndex coord, Vec3f& voxel) {
  voxel.setValue(coord / size_yz_, (coord / size_z_) % size_y_, coord % size_z_);
}

inline VoxelIndex Voxelizer::ConvVoxelToIndex(const Vec3f& voxel) {
  return INDEX(voxel[0], voxel[1], voxel[2]);
}

inline void Voxelizer::RandomPermutation(const V3SP& data, int num) {
  for (int i = 0, id; i < num; ++i) {
    id = Random(i, num - 1);
    if (i != id) std::swap((data.get())[i], (data.get())[id]);
  }
}

inline void Voxelizer::FillYZ(const int x) {
  const int ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  VoxelIndex voxel_index, tmp;
  for (int y = ly, z; y <= uy; ++y) {
    for (z = lz; z <= uz; ++z) {
      voxel_index = INDEX(x, y, x);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
    if (z == uz + 1) continue;
    for (z = uz; z >= lz; --z) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
  }
}

inline void Voxelizer::FillYZ2(const int x) {
  const int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0],
            ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  int nx, ny;
  VoxelIndex voxel_index, tmp;
  for (int y = ly, z; y <= uy; ++y) {
    for (z = lz; z <= uz; ++z) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          ny = y + DI_4[i][1];
          if (nx >= lx && nx <= ux && ny >= ly && ny <= uy) {
            voxel_index = INDEX(nx, ny, z);
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
    if (z == uz + 1) continue;
    for (z = uz; z >= lz; --z) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          ny = y + DI_4[i][1];
          if (nx >= lx && nx <= ux && ny >= ly && ny <= uy) {
            voxel_index = INDEX(nx, ny, z);
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
  }
}

inline void Voxelizer::FillXZ(const int y) {
  const int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  VoxelIndex voxel_index, tmp;
  for (int z = lz, x; z <= uz; ++z) {
    for (x = lx; x <= ux; ++x) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
    if (x == ux + 1) continue;
    for (x = ux; x >= lx; --x) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
  }
}

inline void Voxelizer::FillXZ2(const int y) {
  const int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0],
            ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  int ny, nz;
  VoxelIndex voxel_index, tmp;
  for (int z = lz, x; z <= uz; ++z) {
    for (x = lx; x <= ux; ++x) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          ny = y + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && ny >= ly && ny <= uy) {
            voxel_index = INDEX(x, ny, nz);
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
    if (x == ux + 1) continue;
    for (x = ux; x >= lx; --x) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          ny = y + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && ny >= ly && ny <= uy) {
            voxel_index = INDEX(x, ny, nz);
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
  }
}

inline void Voxelizer::FillXY(const int z) {
  const int ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0];
  VoxelIndex voxel_index, tmp;
  for (int x = lx, y; x <= ux; ++x) {
    for (y = ly; y <= uy; ++y) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
    if (y == uy + 1) continue;
    for (y = uy; y >= ly; --y) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
  }
}

inline void Voxelizer::FillXY2(const int z) {
  const int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0],
            ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  int nx, nz;
  VoxelIndex voxel_index, tmp;
  for (int x = lx, y; x <= ux; ++x) {
    for (y = ly; y <= uy; ++y) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && nx >= lx && nx <= ux) {
            voxel_index = INDEX(nx, y, nz);
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
    if (y == uy + 1) continue;
    for (y = uy; y >= ly; --y) {
      voxel_index = INDEX(x, y, z);
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && nx >= lx && nx <= ux) {
            voxel_index = INDEX(nx, y, nz);
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
  }
}

inline void Voxelizer::RunSolidTask(size_t num_thread) {
  ThreadPool tp(num_thread);
  const int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0],
            ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  for (int x = lx; x <= ux; ++x) {
    tp.Run(boost::bind(&Voxelizer::FillYZ, this, x));
  }
  for (int y = ly; y <= uy; ++y) {
    tp.Run(boost::bind(&Voxelizer::FillXZ, this, y));
  }
  for (int z = lz; z <= uz; ++z) {
    tp.Run(boost::bind(&Voxelizer::FillXY, this, z));
  }
  tp.Stop();
}

inline void Voxelizer::RunSolidTask2(size_t num_thread) {
  ThreadPool tp(num_thread);
  const int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0],
            ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1],
            lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  for (int x = lx; x <= ux; ++x) {
    tp.Run(boost::bind(&Voxelizer::FillYZ2, this, x));
  }
  for (int z = lz; z <= uz; ++z) {
    tp.Run(boost::bind(&Voxelizer::FillXY2, this, z));
  }
  for (int y = ly; y <= uy; ++y) {
    tp.Run(boost::bind(&Voxelizer::FillXZ2, this, y));
  }
  tp.Stop();
}

/**
 * Write to file with a output format
 */
void Voxelizer::Write(const std::string& p_file, const std::string& format) {
  if (format == "binvox") {
    WriteBinvox(p_file);
  } else if (format == "rawvox") {
    WriteRawvox(p_file);
  } else if (format == "cmpvox") {
    WriteCmpvox(p_file);
  } else {
    std::cout << "no such format: " << format << std::endl;
  }
}

/**
 * Write to file with compression.
 */
void Voxelizer::WriteCmpvox(const std::string& p_file) {
  if (verbose_) std::cout << "writing voxels to file..." << std::endl;
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  std::ofstream* output = new std::ofstream(p_file.c_str(), std::ios::out | std::ios::binary);

  //
  // write header
  //
  *output << size_x_ << size_y_ << size_z_;
  *output << (double)(*lb_)[0] << (double)(*lb_)[1] << (double)(*lb_)[2];

  *output << (double)(*unit_)[0] << (double)(*unit_)[1] << (double)(*unit_)[2];
  *output << lx << ly << lz << uz << uy << uz;

  if (verbose_) std::cout << "grid size : " << size_x_ << ", " << size_y_ << ", " << size_z_ << std::endl;
  if (verbose_)
    std::cout << "lower bound : " << (*lb_)[0] << " " << (*lb_)[1] << " "
         << (*lb_)[2] << std::endl;
  if (verbose_) std::cout << "voxel size : " << (*unit_)[0] * 2 << std::endl;
  if (verbose_)
    std::cout << "voxel bound : (" << lx << " " << ly << " " << lz << "), "
         << " (" << ux << " " << uy << " " << uz << ")" << std::endl;

  //
  // write data
  //
  /**
   * Compression
   */
  VoxelIndex index, total_ones = 0;
  int x = lx, y = ly, z = lz;
  Byte value, count;
  while (x <= ux) {
    index = INDEX(x, y, z);
    value = GETBIT(voxels_.get()[index / kBatchSize], index);
    count = 0;
    while ((x <= ux) && (count < 255) &&
           (value == GETBIT(voxels_.get()[index / kBatchSize], index))) {
      z++;
      if (z > uz) {
        z = lz;
        y++;
        if (y > uy) {
          y = ly;
          x++;
        }
      }
      index = INDEX(x, y, z);
      count++;
    }
    if (value) total_ones += count;
    *output << value << count;
  }

  output->close();
  if (verbose_) std::cout << "wrote " << total_ones << " voxels" << std::endl;
}

/**
 * Write to file, with binvox format, check https://www.patrickmin.com/viewvox/
 */
void Voxelizer::WriteBinvox(const std::string& p_file) {
  if (verbose_) std::cout << "writing voxels to file..." << std::endl;

  V3SP vxlBox(new Vec3f(0, 0, 0));
  const int lx = 0, ux = size_x_ - 1, ly = 0, uy = size_y_ - 1, lz = 0, uz = size_z_ - 1;
  const int bx = ux - lx + 1, by = uy - ly + 1, bz = uz - lz + 1;
  const int mesh_lx = (*mesh_vox_lb_)[0], mesh_ly = (*mesh_vox_lb_)[1],
      mesh_lz = (*mesh_vox_lb_)[2];
  const int mesh_ux = (*mesh_vox_ub_)[0], mesh_uy = (*mesh_vox_ub_)[1],
      mesh_uz = (*mesh_vox_ub_)[2];

  std::ofstream* output = new std::ofstream(p_file.c_str(), std::ios::out | std::ios::binary);

  Vec3f& norm_translate = (*lb_);
  float norm_scale = (*bound_).norm();

  //
  // write header
  //
  *output << "#binvox 1" << std::endl;
  *output << "dim " << bx << " " << by << " " << bz << std::endl;
  if (verbose_) std::cout << "dim : " << bx << " x " << by << " x " << bz << std::endl;
  *output << "translate " << -norm_translate[0] << " " << -norm_translate[2]
          << " " << -norm_translate[1] << std::endl;
  *output << "scale " << norm_scale << std::endl;
  *output << "data" << std::endl;

  Byte value;
  Byte count;
  VoxelIndex index = 0, total_ones = 0;
  int bytes_written = 0;

  /**
   * Compression
   */
  int x = lx, y = ly, z = lz;
  while (x <= ux) {
    index = INDEX(x, y, z);
    value =
        InRange(x, y, z, mesh_lx, mesh_ly, mesh_lz, mesh_ux, mesh_uy, mesh_uz)
            ? GETBIT(voxels_.get()[index / kBatchSize], index)
            : 0;
    count = 0;
    while ((x <= ux) && (count < 255) &&
           (value == (InRange(x, y, z, mesh_lx, mesh_ly, mesh_lz, mesh_ux,
                              mesh_uy, mesh_uz)
                          ? GETBIT(voxels_.get()[index / kBatchSize], index)
                          : 0))) {
      z++;
      if (z > uz) {
        z = lz;
        y++;
        if (y > uy) {
          y = ly;
          x++;
        }
      }
      index = INDEX(x, y, z);
      count++;
    }
    if (value) total_ones += count;
    *output << value << count;
    bytes_written += 2;
  }

  output->close();
  if (verbose_)
    std::cout << "wrote " << total_ones << " set voxels out of " << static_cast<VoxelIndex>(bx) * by * bz
         << ", in " << bytes_written << " bytes" << std::endl;
}

/**
 * Write to file with raw format.
 */
void Voxelizer::WriteRawvox(const std::string& p_file) {
  if (verbose_) std::cout << "writing voxels to file..." << std::endl;
  int lx = 0, ux = size_x_ - 1, ly = 0, uy = size_y_ - 1, lz = 0, uz = size_z_ - 1;
  int mesh_lx = (*mesh_vox_lb_)[0], mesh_ly = (*mesh_vox_lb_)[1],
      mesh_lz = (*mesh_vox_lb_)[2];
  int mesh_ux = (*mesh_vox_ub_)[0], mesh_uy = (*mesh_vox_ub_)[1],
      mesh_uz = (*mesh_vox_ub_)[2];
  std::ofstream* output = new std::ofstream(p_file.c_str(), std::ios::out | std::ios::binary);

  //
  // write header
  //
  *output << size_x_ << " " << size_y_ << " " << size_z_ << std::endl;
  *output << (double)(*lb_)[0] << " " << (double)(*lb_)[1] << " "
          << (double)(*lb_)[2] << std::endl;
  *output << (double)(*unit_)[0] << (double)(*unit_)[1] << (double)(*unit_)[2] << std::endl;

  if (verbose_)
    std::cout << "dim : " << size_x_ << " x " << size_y_ << " x " << size_z_ << std::endl;
  if (verbose_) std::cout << "lower bound : " << (*lb_) << std::endl;
  if (verbose_) std::cout << "voxel size : " << (*unit_)[0] << " " << (*unit_)[1] << " " << (*unit_)[2] << std::endl;

  //
  // write data
  //
  VoxelIndex voxel_index, tmp, count = 0;
  for (int x = lx; x <= ux; ++x) {
    for (int y = ly; y <= uy; ++y) {
      for (int z = lz; z <= uz; ++z) {
        if (!InRange(x, y, z, mesh_lx, mesh_ly, mesh_lz, mesh_ux, mesh_uy,
                     mesh_uz))
          continue;
        voxel_index = INDEX(x, y, z);
        tmp = (voxels_.get())[voxel_index / kBatchSize].load();
        if (GETBIT(tmp, voxel_index)) {
          *output << x << ' ' << y << ' ' << z << '\n';
          // if (count == 0) std::cout << x << " " << y << " " << z << std::endl;
          ++count;
        }
      }
    }
  }
  output->close();
  if (verbose_) std::cout << "wrote " << count << " voxels" << std::endl;
}

Voxelizer::~Voxelizer() {
  // TODO Auto-generated destructor stub
}

V3SP Voxelizer::GetVertices() { return vertices_; }

V3SP Voxelizer::GetFaces() { return faces_; }

int Voxelizer::GetVerticesSize() { return num_vertices_; }

int Voxelizer::GetFacesSize() { return num_faces_; }

Vec3f Voxelizer::GetLowerBound() { return *lb_; }

Vec3f Voxelizer::GetUpperBound() { return *ub_; }

Vec3f Voxelizer::GetMeshLowerBound() { return *mesh_lb_; }

Vec3f Voxelizer::GetMeshUpperBound() { return *mesh_ub_; }

AVISP Voxelizer::GetVoxels() { return voxels_; }

Vec3f Voxelizer::GetHalfUnit() { return *half_unit_; }

Vec3f Voxelizer::GetUnit() { return *unit_; }

VoxelIndex Voxelizer::GetTotalSize() { return compressed_total_size_; }

}  // namespace voxelizer
