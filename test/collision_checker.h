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

typedef unique_ptr<voxelizer::Voxelizer> VoxUP;
typedef boost::shared_ptr<CollisionObject> COSP;
typedef unique_ptr<CollisionObject> COUP;
typedef fcl::BVHModel<OBBRSS> Model;
typedef boost::shared_ptr<Model> ModelSP;
typedef boost::shared_ptr<unsigned int> UintSP;

class CollisionChecker {
  void EulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R);
  COSP GenRandomCO(double ratio);
  void PreMeshCO();
  FCL_REAL RandInterval(FCL_REAL rmin, FCL_REAL rmax);
  FCL_REAL extents_[6];
  void GenRandomTransform(FCL_REAL extents[6], Transform3f& transform);
  VoxUP voxelizer_;
  COUP mesh_co_;
  vector<Vec3f> vertices_;
  vector<Triangle> triangles_;
  int size_, size2_, num_thread_;
  string p_file_;
  UintSP voxels_;
  voxelizer::BoxSP unit_;

 public:
  inline bool TestVoxel(const COSP& cube_co);
  inline bool TestMesh(const COSP& cube_co);
  void Test(int num_cases, double ratio);
  bool Init();
  CollisionChecker(int size, int num_thread, const string& p_file)
      : size_(size),
        num_thread_(num_thread),
        p_file_(p_file),
        voxelizer_(new voxelizer::Voxelizer(size, p_file, false)) {}
  virtual ~CollisionChecker();
};

}  // namespace collision_checker

#endif /* COLLISIONCHECKER_H_ */
