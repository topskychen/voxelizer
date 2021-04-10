/*
 * collisionChecker.h
 *
 *  Created on: 30 Jun, 2014
 *      Author: chenqian
 */

#ifndef COLLISIONCHECKER_H_
#define COLLISIONCHECKER_H_

#include <cmath>
#include <cstddef>
#include <cstdio>

#include "commons.h"
#include "fcl/BV/BV.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/collision.h"
#include "fcl/math/transform.h"
#include "fcl/shape/geometric_shapes.h"
#include "time.h"
#include "voxelizer.h"

namespace collision_checker {

typedef std::unique_ptr<voxelizer::Voxelizer> VoxUP;
typedef boost::shared_ptr<CollisionObject> COSP;
typedef std::unique_ptr<CollisionObject> COUP;
typedef fcl::BVHModel<OBBRSS> Model;
typedef boost::shared_ptr<Model> ModelSP;

class CollisionChecker {
  void EulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R);
  COSP GenRandomCO(double ratio);
  void PreMeshCO();
  float RandInterval(float rmin, float rmax);
  FCL_REAL extents_[6];
  void GenRandomTransform(FCL_REAL extents[6], Transform3f& transform);
  VoxUP voxelizer_;
  COUP mesh_co_;
  std::vector<Vec3f> vertices_;
  std::vector<Triangle> triangles_;
  int size_, size2_, num_thread_;
  std::string p_file_;
  voxelizer::VISP voxels_;
  voxelizer::BoxSP unit_;

 public:
  inline bool TestVoxel(const COSP& cube_co);
  inline bool TestMesh(const COSP& cube_co);
  void Test(int num_cases, double ratio);
  bool Init();
  CollisionChecker(int size, int num_thread, const std::string& p_file)
      : size_(size),
        num_thread_(num_thread) {
          voxelizer::Option option;
          option.SetInFilePath(p_file);
          voxelizer_ = absl::make_unique<voxelizer::Voxelizer>(size, option);
        }
  virtual ~CollisionChecker();
};

}  // namespace collision_checker

#endif /* COLLISIONCHECKER_H_ */
