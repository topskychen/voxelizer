#include "ray_caster.h"

#include "fcl/collision.h"
#include "fcl/BV/BV.h"
#include "fcl/BVH/BVH_model.h"

namespace voxelizer {

const Vec3f kFarPoint = Vec3f(-1e5, 1e7, -1e6);
const Triangle kSingleTraingle = Triangle(0, 1, 2);
const int kMaxNumContact = 10;
typedef fcl::BVHModel<OBBRSS> Model;

using fcl::collide;

bool RayCaster::Init() {
  boost::shared_ptr<Model> model(new Model());
  model->beginModel(triangles_.size(), vertices_.size());
  model->addSubModel(vertices_, triangles_);
  model->endModel();
  mesh_co_.reset(new CollisionObject(model));
  mesh_co_->computeAABB();
  return true;
}

int RayCaster::RayCast(const Vec3f& source) {
  boost::shared_ptr<Model> model(new Model());
  model->beginModel(1, 3);
  std::vector<Vec3f> vertices = {source, kFarPoint, kFarPoint};
  std::vector<Triangle> triangles = {kSingleTraingle};
  model->addSubModel(vertices, triangles);
  model->endModel();
  CollisionObject* ray_co = new CollisionObject(model);
  CollisionRequest request(kMaxNumContact, true);
  CollisionResult result;
  if (collide(mesh_co_.get(), ray_co, request, result)) 
    return result.numContacts();
  return 0;
}

bool RayCaster::Inside(const Vec3f& source) {
  const int num_contacts = RayCast(source);
  return num_contacts % 2;
}

}