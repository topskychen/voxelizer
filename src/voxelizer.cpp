/*
 * Voxelizer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include <exception>
#include <boost/format.hpp>

#include "voxelizer.h"

namespace voxelizer {

bool Voxelizer::Init() {
  if (verbose_) cout << "voxelizer init... " << endl;
  is_init_ = false;
  const aiScene* scene;
  try {
    /**
     * Reset voxels.
     */
    voxels_.reset(new AUint[total_size_], ArrayDeleter<AUint>());
    voxels_buffer_.reset(new AUint[total_size_], ArrayDeleter<AUint>());
    memset(voxels_.get(), 0, total_size_ * sizeof(int));
    memset(voxels_buffer_.get(), 0, total_size_ * sizeof(int));

    /*
     * Load scene
     * */
    Assimp::Importer importer;
    scene = importer.ReadFile(p_file_, aiProcessPreset_TargetRealtime_Quality | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes);
    if (!scene) {
      throw std::runtime_error("Scene fails to be loaded!");
    }

    if (verbose_) cout << "mesh number: " << scene->mNumMeshes << endl;

    if (scene->mNumMeshes == 0) {
      throw std::runtime_error("0 mesh in the scene!");         
    }

    if (scene->mNumMeshes <= mesh_index_) {
      throw std::runtime_error("Required mesh index out of range!");          
    }

    // TODO(topskychen@gmail.com): consider all the meshes when mesh_index_ is -1.
    aiMesh* mesh = scene->mMeshes[mesh_index_];

    /**
     * Store info.
     */
    num_vertices_ = mesh->mNumVertices;
    num_faces_ = mesh->mNumFaces;
    if (verbose_) cout << "faces : " << num_faces_ << std::endl;
    if (verbose_) cout << "vertices : " << num_vertices_ << std::endl;
    LoadFromMesh(mesh);

    if (!scene) delete scene;
    is_init_ = true;
  } catch (std::exception& e) {
    cout << e.what() << endl;
    if (!scene) delete scene;
    return false;
  }
  if (verbose_) cout << "done." << endl;

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

  mesh_lb_.reset(new Vec3f((*mesh_lb_) - Vec3f(0.0001, 0.0001, 0.0001)));
  mesh_ub_.reset(new Vec3f((*mesh_ub_) + Vec3f(0.0001, 0.0001, 0.0001)));

  /**
   *
   */
  min_lb_ = (*mesh_lb_)[0];
  min_lb_ = min(min_lb_, (float)(*mesh_lb_)[1]);
  min_lb_ = min(min_lb_, (float)(*mesh_lb_)[2]);
  max_ub_ = (*mesh_ub_)[0];
  max_ub_ = max(max_ub_, (float)(*mesh_ub_)[1]);
  max_ub_ = max(max_ub_, (float)(*mesh_ub_)[2]);
  lb_.reset(new Vec3f(min_lb_, min_lb_, min_lb_));
  ub_.reset(new Vec3f(max_ub_, max_ub_, max_ub_));
  bound_.reset(new Vec3f((*ub_ - *lb_)));

  faces_.reset(new Vec3f[num_faces_], ArrayDeleter<Vec3f>());
  if (mesh->mPrimitiveTypes != kPrimitiveTriangleType) {
    throw std::runtime_error(boost::str(boost::format("Mesh face primitive type expects %d indices but received %d.")%kPrimitiveTriangleType%mesh->mPrimitiveTypes));
  }
  for (size_t i = 0; i < num_faces_; ++i) {
    if (mesh->mFaces[i].mNumIndices != kTriangleNumIndices) {
      throw std::runtime_error(boost::str(boost::format("Triangle face expects %d indices but received %d.")%kTriangleNumIndices%mesh->mFaces[i].mNumIndices));
    }
    faces_.get()[i] =
        Vec3f(mesh->mFaces[i].mIndices[0], mesh->mFaces[i].mIndices[1],
              mesh->mFaces[i].mIndices[2]);
  }
  RandomPermutation(faces_, num_faces_);

  Vec3f half_unit = (*bound_) / ((float)size_ * 2);
  half_unit_.reset(new Vec3f(half_unit));
  mesh_vox_lb_ = GetVoxel(mesh_lb_);
  mesh_vox_ub_ = GetVoxel(mesh_ub_);

  if (verbose_) cout << "space : " << *lb_ << ", " << *ub_ << endl;
  if (verbose_)
    cout << "mesh bound : " << *mesh_lb_ << ", " << *mesh_ub_ << endl;
}

/**
 * voxelize the surface.
 */
void Voxelizer::VoxelizeSurface(const int num_thread) {
  if (!is_init_) {
    return;
  }
  if (verbose_) cout << "surface voxelizing... " << endl;
  ThreadPool tp(num_thread);
  for (int i = 0; i < num_faces_; ++i) {
    tp.Run(boost::bind(&Voxelizer::RunSurfaceTask, this, i));
  }
  tp.Stop();
  if (verbose_) cout << "done." << endl;
}

/**
 * Details of surface task.
 */
inline void Voxelizer::RunSurfaceTask(const int tri_id) {
  TriSP tri = GetTri(tri_id);
  tri->computeLocalAABB();
  const V3SP lb = GetVoxel(tri->aabb_local.min_);
  const V3SP ub = GetVoxel(tri->aabb_local.max_);
  int lx = (*lb)[0], ux = (*ub)[0], ly = (*lb)[1], uy = (*ub)[1], lz = (*lb)[2],
      uz = (*ub)[2];

  /**
   * when the estimated voxels are too large, optimize with bfs.
   */
  int count = 0;
  int esti = min(ux - lx, min(uy - ly, uz - lz));
  unsigned int voxel_int, tmp;
  if (esti < 100) {
    V3SP vxlBox(new Vec3f(0, 0, 0));
    for (int x = lx, y, z; x <= ux; ++x) {
      for (y = ly; y <= uy; ++y) {
        for (z = lz; z <= uz; ++z) {
          voxel_int = x * size2_ + y * size_ + z;
          tmp = (voxels_.get())[voxel_int / kBatchSize].load();
          if (GETBIT(tmp, voxel_int)) continue;
          vxlBox->setValue(x, y, z);
          if (Collide(half_unit_, GetLoc(vxlBox), tri)) {
            (voxels_.get())[voxel_int / kBatchSize] |=
                (1 << (voxel_int % kBatchSize));
            count++;
          }
        }
      }
    }
  } else {
    count = BfsSurface(tri, lb, ub);
  }
}

inline int Voxelizer::BfsSurface(const TriSP& tri, const V3SP& lb,
                                 const V3SP& ub) {
  queue<unsigned int> q;
  HashSet set;
  unsigned int start = ConvVoxelToInt(GetVoxel(tri->a)), top_voxel_int, tmp,
               new_voxel_int;
  q.push(start);
  set.insert(start);
  V3SP top_voxel;
  int count = 0;
  while (!q.empty()) {
    count++;
    top_voxel_int = q.front();
    q.pop();
    tmp = (voxels_.get())[top_voxel_int / kBatchSize].load();
    top_voxel = ConvIntToVoxel(top_voxel_int);
    if (GETBIT(tmp, top_voxel_int) ||
        Collide(half_unit_, GetLoc(top_voxel), tri)) {
      if (!GETBIT(tmp, top_voxel_int)) {
        (voxels_.get())[top_voxel_int / kBatchSize] |=
            (1 << (top_voxel_int % kBatchSize));
      }
      for (int i = 0; i < 6; ++i) {
        Vec3f newVoxel = *top_voxel + D_6[i];
        if (!InRange(newVoxel, lb, ub)) continue;
        new_voxel_int = ConvVoxelToInt(newVoxel);
        if (set.find(new_voxel_int) == set.end()) {
          set.insert(new_voxel_int);
          q.push(new_voxel_int);
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
  if (verbose_) cout << "solid voxelizing... " << endl;
  if (verbose_) cout << "round 1..." << endl;
  RunSolidTask(num_thread);
  if (verbose_) cout << "round 2..." << endl;
  RunSolidTask2(num_thread);
  for (int i = 0; i < total_size_; ++i)
    voxels_.get()[i] = voxels_buffer_.get()[i] ^ (~0);
  if (verbose_) cout << "done." << endl;
}

inline void Voxelizer::BfsSolid(const unsigned int start_int) {
  unsigned int voxel_int = start_int,
               tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                     (voxels_buffer_.get())[voxel_int / kBatchSize].load();
  if (GETBIT(tmp, voxel_int)) return;
  queue<unsigned int> q;
  q.push(voxel_int);
  V3SP top_voxel(new Vec3f(0, 0, 0));
  while (!q.empty()) {
    voxel_int = q.front();
    tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
          (voxels_buffer_.get())[voxel_int / kBatchSize].load();
    q.pop();
    top_voxel = ConvIntToVoxel(voxel_int);
    if (!GETBIT(tmp, voxel_int)) {
      (voxels_buffer_.get())[voxel_int / kBatchSize] |=
          (1 << (voxel_int % kBatchSize));
      for (int i = 0; i < 6; i++) {
        Vec3f newVoxel = *top_voxel + D_6[i];
        if (!InRange(newVoxel, mesh_vox_lb_, mesh_vox_ub_)) continue;
        voxel_int = ConvVoxelToInt(newVoxel);
        tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
              (voxels_buffer_.get())[voxel_int / kBatchSize].load();
        if (!GETBIT(tmp, voxel_int)) q.push(voxel_int);
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

inline V3SP Voxelizer::ConvIntToVoxel(const unsigned int& coord) {
  V3SP voxel(new Vec3f(coord / size2_, (coord / size_) % size_, coord % size_));
  return voxel;
}

inline unsigned int Voxelizer::ConvVoxelToInt(const V3SP& voxel) {
  return (*voxel)[0] * size2_ + (*voxel)[1] * size_ + (*voxel)[2];
}

inline unsigned int Voxelizer::ConvVoxelToInt(const Vec3f& voxel) {
  return voxel[0] * size2_ + voxel[1] * size_ + voxel[2];
}

inline void Voxelizer::RandomPermutation(const V3SP& data, int num) {
  for (int i = 0, id; i < num; ++i) {
    id = Random(i, num - 1);
    if (i != id) swap((data.get())[i], (data.get())[id]);
  }
}

inline void Voxelizer::FillYZ(const int x) {
  int ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2],
      uz = (*mesh_vox_ub_)[2];
  unsigned int voxel_int, tmp;
  for (int y = ly, z; y <= uy; ++y) {
    for (z = lz; z <= uz; ++z) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        (voxels_buffer_.get())[voxel_int / kBatchSize] |=
            (1 << (voxel_int % kBatchSize));
      }
    }
    if (z == uz + 1) continue;
    for (z = uz; z >= lz; --z) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        (voxels_buffer_.get())[voxel_int / kBatchSize] |=
            (1 << (voxel_int % kBatchSize));
      }
    }
  }
}

inline void Voxelizer::FillYZ2(const int x) {
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2],
      nx, ny;
  unsigned int voxel_int, tmp;
  for (int y = ly, z; y <= uy; ++y) {
    for (z = lz; z <= uz; ++z) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          ny = y + DI_4[i][1];
          if (nx >= lx && nx <= ux && ny >= ly && ny <= uy) {
            voxel_int = nx * size2_ + ny * size_ + z;
            tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_int / kBatchSize].load();
            if (!GETBIT(tmp, voxel_int)) BfsSolid(voxel_int);
          }
        }
      }
    }
    if (z == uz + 1) continue;
    for (z = uz; z >= lz; --z) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          ny = y + DI_4[i][1];
          if (nx >= lx && nx <= ux && ny >= ly && ny <= uy) {
            voxel_int = nx * size2_ + ny * size_ + z;
            tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_int / kBatchSize].load();
            if (!GETBIT(tmp, voxel_int)) BfsSolid(voxel_int);
          }
        }
      }
    }
  }
}

inline void Voxelizer::FillXZ(const int y) {
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], lz = (*mesh_vox_lb_)[2],
      uz = (*mesh_vox_ub_)[2];
  unsigned int voxel_int, tmp;
  for (int z = lz, x; z <= uz; ++z) {
    for (x = lx; x <= ux; ++x) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        (voxels_buffer_.get())[voxel_int / kBatchSize] |=
            (1 << (voxel_int % kBatchSize));
      }
    }
    if (x == ux + 1) continue;
    for (x = ux; x >= lx; --x) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        (voxels_buffer_.get())[voxel_int / kBatchSize] |=
            (1 << (voxel_int % kBatchSize));
      }
    }
  }
}

inline void Voxelizer::FillXZ2(const int y) {
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2],
      ny, nz;
  unsigned int voxel_int, tmp;
  for (int z = lz, x; z <= uz; ++z) {
    for (x = lx; x <= ux; ++x) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          ny = y + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && ny >= ly && ny <= uy) {
            voxel_int = x * size2_ + ny * size_ + nz;
            tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_int / kBatchSize].load();
            if (!GETBIT(tmp, voxel_int)) BfsSolid(voxel_int);
          }
        }
      }
    }
    if (x == ux + 1) continue;
    for (x = ux; x >= lx; --x) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          ny = y + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && ny >= ly && ny <= uy) {
            voxel_int = x * size2_ + ny * size_ + nz;
            tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_int / kBatchSize].load();
            if (!GETBIT(tmp, voxel_int)) BfsSolid(voxel_int);
          }
        }
      }
    }
  }
}

inline void Voxelizer::FillXY(const int z) {
  int ly = (*mesh_vox_lb_)[1], uy = (*mesh_vox_ub_)[1], lx = (*mesh_vox_lb_)[0],
      ux = (*mesh_vox_ub_)[0];
  unsigned int voxel_int, tmp;
  for (int x = lx, y; x <= ux; ++x) {
    for (y = ly; y <= uy; ++y) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        (voxels_buffer_.get())[voxel_int / kBatchSize] |=
            (1 << (voxel_int % kBatchSize));
      }
    }
    if (y == uy + 1) continue;
    for (y = uy; y >= ly; --y) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        (voxels_buffer_.get())[voxel_int / kBatchSize] |=
            (1 << (voxel_int % kBatchSize));
      }
    }
  }
}

inline void Voxelizer::FillXY2(const int z) {
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2],
      nx, nz;
  unsigned int voxel_int, tmp;
  for (int x = lx, y; x <= ux; ++x) {
    for (y = ly; y <= uy; ++y) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && nx >= lx && nx <= ux) {
            voxel_int = nx * size2_ + y * size_ + nz;
            tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_int / kBatchSize].load();
            if (!GETBIT(tmp, voxel_int)) BfsSolid(voxel_int);
          }
        }
      }
    }
    if (y == uy + 1) continue;
    for (y = uy; y >= ly; --y) {
      voxel_int = x * size2_ + y * size_ + z;
      tmp = (voxels_.get())[voxel_int / kBatchSize].load();
      if (GETBIT(tmp, voxel_int)) {
        break;
      } else {
        for (int i = 0; i < 4; ++i) {
          nx = x + DI_4[i][0];
          nz = z + DI_4[i][1];
          if (nz >= lz && nz <= uz && nx >= lx && nx <= ux) {
            voxel_int = nx * size2_ + y * size_ + nz;
            tmp = (voxels_.get())[voxel_int / kBatchSize].load() |
                  (voxels_buffer_.get())[voxel_int / kBatchSize].load();
            if (!GETBIT(tmp, voxel_int)) BfsSolid(voxel_int);
          }
        }
      }
    }
  }
}

inline void Voxelizer::RunSolidTask(size_t num_thread) {
  ThreadPool tp(num_thread);
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
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
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
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
void Voxelizer::Write(const string& p_file, const string& format) {
  if (format == "binvox") {
    WriteBinvox(p_file);
  } else if (format == "rawvox") {
    WriteRawvox(p_file);
  } else if (format == "cmpvox") {
    WriteCmpvox(p_file);
  } else {
    cout << "no such format: " << format << endl;
  }
}

/**
 * Write to file with compression.
 */
void Voxelizer::WriteCmpvox(const string& p_file) {
  if (verbose_) cout << "writing voxels to file..." << endl;
  int lx = (*mesh_vox_lb_)[0], ux = (*mesh_vox_ub_)[0], ly = (*mesh_vox_lb_)[1],
      uy = (*mesh_vox_ub_)[1], lz = (*mesh_vox_lb_)[2], uz = (*mesh_vox_ub_)[2];
  ofstream* output = new ofstream(p_file.c_str(), ios::out | ios::binary);

  //
  // write header
  //
  *output << size_;
  *output << (double)(*lb_)[0] << (double)(*lb_)[1] << (double)(*lb_)[2];

  *output << (double)(*half_unit_)[0] * 2;
  *output << lx << ly << lz << uz << uy << uz;

  if (verbose_) cout << "grid size : " << size_ << endl;
  if (verbose_)
    cout << "lower bound : " << (*lb_)[0] << " " << (*lb_)[1] << " "
         << (*lb_)[2] << endl;
  if (verbose_) cout << "voxel size : " << (*half_unit_)[0] * 2 << endl;
  if (verbose_)
    cout << "voxel bound : (" << lx << " " << ly << " " << lz << "), "
         << " (" << ux << " " << uy << " " << uz << ")" << endl;

  //
  // write data
  //
  /**
   * Compression
   */
  int x = lx, y = ly, z = lz, index, total_ones = 0;
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
  if (verbose_) cout << "wrote " << total_ones << " voxels" << endl;
}

/**
 * Write to file, with binvox format, check https://www.patrickmin.com/viewvox/
 */
void Voxelizer::WriteBinvox(const string& p_file) {
  if (verbose_) cout << "writing voxels to file..." << endl;

  V3SP vxlBox(new Vec3f(0, 0, 0));
  int lx = 0, ux = size_ - 1, ly = 0, uy = size_ - 1, lz = 0, uz = size_ - 1;
  int bx = ux - lx + 1, by = uy - ly + 1, bz = uz - lz + 1;
  int mesh_lx = (*mesh_vox_lb_)[0], mesh_ly = (*mesh_vox_lb_)[1],
      mesh_lz = (*mesh_vox_lb_)[2];
  int mesh_ux = (*mesh_vox_ub_)[0], mesh_uy = (*mesh_vox_ub_)[1],
      mesh_uz = (*mesh_vox_ub_)[2];

  ofstream* output = new ofstream(p_file.c_str(), ios::out | ios::binary);

  Vec3f& norm_translate = (*lb_);
  float norm_scale = (*bound_).norm();

  //
  // write header
  //
  *output << "#binvox 1" << endl;
  *output << "dim " << bx << " " << by << " " << bz << endl;
  if (verbose_) cout << "dim : " << bx << " x " << by << " x " << bz << endl;
  *output << "translate " << -norm_translate[0] << " " << -norm_translate[2]
          << " " << -norm_translate[1] << endl;
  *output << "scale " << norm_scale << endl;
  *output << "data" << endl;

  Byte value;
  Byte count;
  int index = 0;
  int bytes_written = 0;
  int total_ones = 0;

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
    cout << "wrote " << total_ones << " set voxels out of " << bx * by * bz
         << ", in " << bytes_written << " bytes" << endl;
}

/**
 * Write to file with raw format.
 */
void Voxelizer::WriteRawvox(const string& p_file) {
  if (verbose_) cout << "writing voxels to file..." << endl;
  int lx = 0, ux = size_ - 1, ly = 0, uy = size_ - 1, lz = 0, uz = size_ - 1;
  int mesh_lx = (*mesh_vox_lb_)[0], mesh_ly = (*mesh_vox_lb_)[1],
      mesh_lz = (*mesh_vox_lb_)[2];
  int mesh_ux = (*mesh_vox_ub_)[0], mesh_uy = (*mesh_vox_ub_)[1],
      mesh_uz = (*mesh_vox_ub_)[2];
  ofstream* output = new ofstream(p_file.c_str(), ios::out | ios::binary);

  //
  // write header
  //
  *output << size_ << endl;
  *output << (double)(*lb_)[0] << " " << (double)(*lb_)[1] << " "
          << (double)(*lb_)[2] << endl;
  *output << (double)(*half_unit_)[0] * 2 << endl;

  if (verbose_)
    cout << "dim : " << size_ << " x " << size_ << " x " << size_ << endl;
  if (verbose_) cout << "lower bound : " << (*lb_) << endl;
  if (verbose_) cout << "voxel size : " << (*half_unit_)[0] * 2 << endl;

  //
  // write data
  //
  unsigned int voxel_int, tmp, count = 0;
  for (int x = lx; x <= ux; ++x) {
    for (int y = ly; y <= uy; ++y) {
      for (int z = lz; z <= uz; ++z) {
        if (!InRange(x, y, z, mesh_lx, mesh_ly, mesh_lz, mesh_ux, mesh_uy,
                     mesh_uz))
          continue;
        voxel_int = x * size2_ + y * size_ + z;
        tmp = (voxels_.get())[voxel_int / kBatchSize].load();
        if (GETBIT(tmp, voxel_int)) {
          *output << x << ' ' << y << ' ' << z << '\n';
          // if (count == 0) cout << x << " " << y << " " << z << endl;
          ++count;
        }
      }
    }
  }
  output->close();
  if (verbose_) cout << "wrote " << count << " voxels" << endl;
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

AUintSP Voxelizer::GetVoxels() { return voxels_; }

V3SP Voxelizer::GetHalfUnit() { return half_unit_; }

int Voxelizer::GetTotalSize() { return total_size_; }

}  // namespace voxelizer
