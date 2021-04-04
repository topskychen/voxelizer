#ifndef RAY_CASTER_H_
#define RAY_CASTER_H_

#include "commons.h"
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "voxelizer.h"

namespace voxelizer {

class RayCaster {
  std::vector<Vec3f> vertices_;
  std::vector<Triangle> triangles_;
  std::unique_ptr<CollisionObject> mesh_co_;

public:
  RayCaster(const std::vector<Vec3f>& vertices, const std::vector<Triangle>& triangles)
    : vertices_(vertices), triangles_(triangles) {}
  bool Init();
  // Cacluates the number of hit points for this casting ray.
  int RayCast(const Vec3f& source);
  // Whether the source inside the mesh object.
  bool Inside(const Vec3f& source);
};

}

#endif