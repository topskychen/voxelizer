/*
 * Voxelizer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include <exception>
#include <sstream>

#include "voxelizer.h"

namespace voxelizer {

bool Voxelizer::Init() {
  if (verbose_) std::cout << "voxelizer init... " << std::endl;
  is_init_ = false;
  const aiScene* scene;
  try {
    /**
     * Reset voxels.
     */
    voxels_.reset(new AVI[compressed_total_size_], ArrayDeleter<AVI>());
    voxels_buffer_.reset(new AVI[compressed_total_size_], ArrayDeleter<AVI>());
    memset(voxels_.get(), 0, compressed_total_size_ * sizeof(VoxelIndex));
    memset(voxels_buffer_.get(), 0, compressed_total_size_ * sizeof(VoxelIndex));

    /*
     * Load scene
     * */
    Assimp::Importer importer;
    scene = importer.ReadFile(p_file_, aiProcessPreset_TargetRealtime_Quality |
                                           aiProcess_OptimizeGraph |
                                           aiProcess_OptimizeMeshes);
    if (!scene) {
      throw std::runtime_error("Scene fails to be loaded!");
    }

    if (verbose_) std::cout << "mesh number: " << scene->mNumMeshes << std::endl;

    if (scene->mNumMeshes == 0) {
      throw std::runtime_error("0 mesh in the scene!");
    }

    if (scene->mNumMeshes <= mesh_index_) {
      throw std::runtime_error("Required mesh index out of range!");
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
    LoadFromMesh(mesh);

    if (!scene) delete scene;
    is_init_ = true;
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
    if (!scene) delete scene;
    return false;
  }
  if (verbose_) std::cout << "done." << std::endl;

  return true;
}

/**
 * Given voxel (int x, int y, int z), return loc (float x, float y, float z)
 */
V3SP Voxelizer::GetLoc(const V3SP& voxel) {
  Vec3f tmp = *lb_ + (*bound_) * (*voxel) / (float)size_;
  V3SP loc(new Vec3f(tmp));
  return loc;
}

V3SP Voxelizer::GetLoc(const Vec3f& voxel) {
  Vec3f tmp = *lb_ + (*bound_) * (voxel) / (float)size_;
  V3SP loc(new Vec3f(tmp));
  return loc;
}

/**
 * Given loc (float x, float y, float z), return voxel (int x, int y, int z)
 */
V3SP Voxelizer::GetVoxel(const Vec3f& loc) {
  Vec3f tmp = (loc - (*lb_)) * (float)size_ / (*bound_);
  V3SP voxel(new Vec3f((int)tmp[0], (int)tmp[1], (int)tmp[2]));
  return voxel;
}

/**
 * Given loc (float x, float y, float z), return voxel (int x, int y, int z)
 */
V3SP Voxelizer::GetVoxel(const V3SP& loc) {
  Vec3f tmp = ((*loc) - (*lb_)) * (float)size_ / (*bound_);
  V3SP voxel(new Vec3f((int)tmp[0], (int)tmp[1], (int)tmp[2]));
  return voxel;
}

/**
 *Get the collision object form triangle(face) id;
 */
inline TriSP Voxelizer::GetTri(const int tri_id) {
  const Vec3f& v_ids = faces_.get()[tri_id];
  TriSP tri(new TriangleP(vertices_.get()[(int)v_ids[0]],
                          vertices_.get()[(int)v_ids[1]],
                          vertices_.get()[(int)v_ids[2]]));
  return tri;
}

/**
 * Load info from mesh.
 */
inline void Voxelizer::LoadFromMesh(const aiMesh* mesh) {
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
   *
   */
  min_lb_ = (*mesh_lb_)[0];
  min_lb_ = std::min(min_lb_, (float)(*mesh_lb_)[1]);
  min_lb_ = std::min(min_lb_, (float)(*mesh_lb_)[2]);
  max_ub_ = (*mesh_ub_)[0];
  max_ub_ = std::max(max_ub_, (float)(*mesh_ub_)[1]);
  max_ub_ = std::max(max_ub_, (float)(*mesh_ub_)[2]);
  lb_.reset(new Vec3f(min_lb_, min_lb_, min_lb_));
  ub_.reset(new Vec3f(max_ub_, max_ub_, max_ub_));
  bound_.reset(new Vec3f((*ub_ - *lb_)));

  faces_.reset(new Vec3f[num_faces_], ArrayDeleter<Vec3f>());
  if (mesh->mPrimitiveTypes != kPrimitiveTriangleType) {
    std::ostringstream sstream;
    sstream << "mesh face primitive type expects " << kPrimitiveTriangleType
            << " indices but received " << mesh->mPrimitiveTypes;
    throw std::runtime_error(sstream.str());
  }
  for (size_t i = 0; i < num_faces_; ++i) {
    if (mesh->mFaces[i].mNumIndices != kTriangleNumIndices) {
      std::ostringstream sstream;
      sstream << "triangle face expects " << kTriangleNumIndices
              << " indices but received " << mesh->mFaces[i].mNumIndices;
      throw std::runtime_error(sstream.str());
    }
    faces_.get()[i] =
        Vec3f(mesh->mFaces[i].mIndices[0], mesh->mFaces[i].mIndices[1],
              mesh->mFaces[i].mIndices[2]);
  }
  RandomPermutation(faces_, num_faces_);

  Vec3f half_unit = (*bound_) / ((float)size_ * 2);
  half_unit_.reset(new Vec3f(half_unit));

  // The real voxel bounding box should extract the epsilon box.
  mesh_vox_lb_ = GetVoxel(*mesh_lb_ + kEpsBox);
  mesh_vox_ub_ = GetVoxel(*mesh_ub_ - kEpsBox);

  if (verbose_) std::cout << "space: " << *lb_ << ", " << *ub_ << std::endl;
  std::cout << "mesh bound: " << *mesh_lb_ << ", " << *mesh_ub_ << std::endl;
  std::cout << "voxel bound: " << *mesh_vox_lb_ << ", " << *mesh_vox_ub_ << std::endl;
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
  TriSP tri = GetTri(tri_id);
  tri->computeLocalAABB();
  const V3SP lb = GetVoxel(tri->aabb_local.min_);
  const V3SP ub = GetVoxel(tri->aabb_local.max_);
  
  const int lx = (*lb)[0], ux = (*ub)[0], ly = (*lb)[1], uy = (*ub)[1], lz = (*lb)[2],
      uz = (*ub)[2];
  /**
   * when the estimated voxels are too large, optimize with bfs.
   */
  VoxelIndex count = 0;
  int esti = std::min(ux - lx, std::min(uy - ly, uz - lz));
  if (esti < kThresholdBfsSurface) {
    VoxelIndex voxel_index, tmp;
    V3SP vxlBox(new Vec3f(0, 0, 0));
    for (int x = lx, y, z; x <= ux; ++x) {
      for (y = ly; y <= uy; ++y) {
        for (z = lz; z <= uz; ++z) {
          voxel_index = x * size2_ + y * size_ + z;
          tmp = (voxels_.get())[voxel_index / kBatchSize].load();
          if (GETBIT(tmp, voxel_index)) continue;
          vxlBox->setValue(x, y, z);
          if (Collide(half_unit_, GetLoc(vxlBox), tri)) {
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

inline VoxelIndex Voxelizer::BfsSurface(const TriSP& tri, const V3SP& lb,
                                 const V3SP& ub) {
  std::queue<VoxelIndex> q;
  HashSet set;
  VoxelIndex start = ConvVoxelToIndex(GetVoxel(tri->a)), top_voxel_index, tmp,
               new_voxel_index;
  q.push(start);
  set.insert(start);
  V3SP top_voxel;
  VoxelIndex count = 0;
  while (!q.empty()) {
    count++;
    top_voxel_index = q.front();
    q.pop();
    tmp = (voxels_.get())[top_voxel_index / kBatchSize].load();
    top_voxel = ConvIndexToVoxel(top_voxel_index);
    if (GETBIT(tmp, top_voxel_index) ||
        Collide(half_unit_, GetLoc(top_voxel), tri)) {
      if (!GETBIT(tmp, top_voxel_index)) {
        SETBIT(voxels_, top_voxel_index);
      }
      for (int i = 0; i < 6; ++i) {
        Vec3f newVoxel = *top_voxel + D_6[i];
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
  V3SP top_voxel(new Vec3f(0, 0, 0));
  while (!q.empty()) {
    voxel_index = q.front();
    tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
          (voxels_buffer_.get())[voxel_index / kBatchSize].load();
    q.pop();
    top_voxel = ConvIndexToVoxel(voxel_index);
    if (!GETBIT(tmp, voxel_index)) {
      SETBIT(voxels_buffer_, voxel_index);
      for (int i = 0; i < 6; i++) {
        Vec3f newVoxel = *top_voxel + D_6[i];
        if (!InRange(newVoxel, mesh_vox_lb_, mesh_vox_ub_)) continue;
        voxel_index = ConvVoxelToIndex(newVoxel);
        tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
              (voxels_buffer_.get())[voxel_index / kBatchSize].load();
        if (!GETBIT(tmp, voxel_index)) q.push(voxel_index);
      }
    }
  }
}

inline bool Voxelizer::InRange(const Vec3f& vc, const V3SP& lb,
                               const V3SP& ub) {
  return vc[0] >= (*lb)[0] && vc[0] <= (*ub)[0] && vc[1] >= (*lb)[1] &&
         vc[1] <= (*ub)[1] && vc[2] >= (*lb)[2] && vc[2] <= (*ub)[2];
}

inline bool Voxelizer::InRange(const int& x, const int& y, const int& z,
                               const int& lx, const int& ly, const int& lz,
                               const int& ux, const int& uy, const int& uz) {
  return x >= lx && x <= ux && y >= ly && y <= uy && z >= lz && z <= uz;
}

inline V3SP Voxelizer::ConvIndexToVoxel(const VoxelIndex coord) {
  V3SP voxel(new Vec3f(coord / size2_, (coord / size_) % size_, coord % size_));
  return voxel;
}

inline VoxelIndex Voxelizer::ConvVoxelToIndex(const V3SP& voxel) {
  return (*voxel)[0] * size2_ + (*voxel)[1] * size_ + (*voxel)[2];
}

inline VoxelIndex Voxelizer::ConvVoxelToIndex(const Vec3f& voxel) {
  return voxel[0] * size2_ + voxel[1] * size_ + voxel[2];
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
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
    if (z == uz + 1) continue;
    for (z = uz; z >= lz; --z) {
      voxel_index = x * size2_ + y * size_ + z;
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
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          ny = y + DI_4[i][1];
          if (nx >= lx && nx <= ux && ny >= ly && ny <= uy) {
            voxel_index = nx * size2_ + ny * size_ + z;
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
    if (z == uz + 1) continue;
    for (z = uz; z >= lz; --z) {
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          ny = y + DI_4[i][1];
          if (nx >= lx && nx <= ux && ny >= ly && ny <= uy) {
            voxel_index = nx * size2_ + ny * size_ + z;
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
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
    if (x == ux + 1) continue;
    for (x = ux; x >= lx; --x) {
      voxel_index = x * size2_ + y * size_ + z;
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
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          ny = y + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && ny >= ly && ny <= uy) {
            voxel_index = x * size2_ + ny * size_ + nz;
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
    if (x == ux + 1) continue;
    for (x = ux; x >= lx; --x) {
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          ny = y + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && ny >= ly && ny <= uy) {
            voxel_index = x * size2_ + ny * size_ + nz;
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
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        SETBIT(voxels_buffer_, voxel_index);
      }
    }
    if (y == uy + 1) continue;
    for (y = uy; y >= ly; --y) {
      voxel_index = x * size2_ + y * size_ + z;
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
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && nx >= lx && nx <= ux) {
            voxel_index = nx * size2_ + y * size_ + nz;
            tmp = (voxels_.get())[voxel_index / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_index / kBatchSize].load();
            if (!GETBIT(tmp, voxel_index)) BfsSolid(voxel_index);
          }
        }
      }
    }
    if (y == uy + 1) continue;
    for (y = uy; y >= ly; --y) {
      voxel_index = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_index / kBatchSize].load();
      if (GETBIT(tmp, voxel_index)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && nx >= lx && nx <= ux) {
            voxel_index = nx * size2_ + y * size_ + nz;
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
  *output << size_;
  *output << (double)(*lb_)[0] << (double)(*lb_)[1] << (double)(*lb_)[2];

  *output << (double)(*half_unit_)[0] * 2;
  *output << lx << ly << lz << uz << uy << uz;

  if (verbose_) std::cout << "grid size : " << size_ << std::endl;
  if (verbose_)
    std::cout << "lower bound : " << (*lb_)[0] << " " << (*lb_)[1] << " "
         << (*lb_)[2] << std::endl;
  if (verbose_) std::cout << "voxel size : " << (*half_unit_)[0] * 2 << std::endl;
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
    index = x * size2_ + y * size_ + z;
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
      index = x * size2_ + y * size_ + z;
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
  const int lx = 0, ux = size_ - 1, ly = 0, uy = size_ - 1, lz = 0, uz = size_ - 1;
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
    index = x * size2_ + y * size_ + z;
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
      index = x * size2_ + y * size_ + z;
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
  int lx = 0, ux = size_ - 1, ly = 0, uy = size_ - 1, lz = 0, uz = size_ - 1;
  int mesh_lx = (*mesh_vox_lb_)[0], mesh_ly = (*mesh_vox_lb_)[1],
      mesh_lz = (*mesh_vox_lb_)[2];
  int mesh_ux = (*mesh_vox_ub_)[0], mesh_uy = (*mesh_vox_ub_)[1],
      mesh_uz = (*mesh_vox_ub_)[2];
  std::ofstream* output = new std::ofstream(p_file.c_str(), std::ios::out | std::ios::binary);

  //
  // write header
  //
  *output << size_ << std::endl;
  *output << (double)(*lb_)[0] << " " << (double)(*lb_)[1] << " "
          << (double)(*lb_)[2] << std::endl;
  *output << (double)(*half_unit_)[0] * 2 << std::endl;

  if (verbose_)
    std::cout << "dim : " << size_ << " x " << size_ << " x " << size_ << std::endl;
  if (verbose_) std::cout << "lower bound : " << (*lb_) << std::endl;
  if (verbose_) std::cout << "voxel size : " << (*half_unit_)[0] * 2 << std::endl;

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
        voxel_index = x * size2_ + y * size_ + z;
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

V3SP Voxelizer::GetLowerBound() { return lb_; }

V3SP Voxelizer::GetUpperBound() { return ub_; }

V3SP Voxelizer::GetMeshLowerBound() { return mesh_lb_; }

V3SP Voxelizer::GetMeshUpperBound() { return mesh_ub_; }

AVISP Voxelizer::GetVoxels() { return voxels_; }

V3SP Voxelizer::GetHalfUnit() { return half_unit_; }

VoxelIndex Voxelizer::GetTotalSize() { return compressed_total_size_; }

}  // namespace voxelizer
