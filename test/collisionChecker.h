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

typedef boost::shared_ptr<voxelizer::Voxelizer> vox_p;
typedef boost::shared_ptr<CollisionObject> co_p;
typedef fcl::BVHModel<OBBRSS> Model;
typedef boost::shared_ptr<Model> model_p;
typedef boost::shared_ptr<unsigned int> UintSP;

class CollisionChecker {
  void EulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R);
  co_p GenRandomCO(double ratio);
  void PreMeshCO();
  FCL_REAL RandInterval(FCL_REAL rmin, FCL_REAL rmax);
  FCL_REAL extents_[6];
  void GenRandomTransform(FCL_REAL extents[6], Transform3f& transform);
  vox_p voxelizer_;
  co_p mesh_co_;
  vector<Vec3f> vertices_;
  vector<Triangle> triangles_;
  int size_, size2_;
  UintSP voxels_;
  voxelizer::BoxSP unit_;

 public:
  inline bool TestVoxel(const co_p& cube_co);
  inline bool TestMesh(const co_p& cube_co);
  void Test(int num_cases, double ratio);
  CollisionChecker(int size, int num_thread, const string& p_file);
  virtual ~CollisionChecker();
};

}  // namespace collision_checker

#endif /* COLLISIONCHECKER_H_ */
