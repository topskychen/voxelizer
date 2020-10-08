/*
 * collisionChecker.cpp
 *
 *  Created on: 30 Jun, 2014
 *      Author: chenqian
 */

#include "collision_checker.h"

using voxelizer::ArrayDeleter;
using voxelizer::BoxSP;
using voxelizer::kBatchSize;
using voxelizer::Timer;
using voxelizer::V3SP;
using voxelizer::Voxelizer;
using voxelizer::VoxelIndex;

namespace collision_checker {

FCL_REAL CollisionChecker::RandInterval(FCL_REAL rmin, FCL_REAL rmax) {
  FCL_REAL t = rand() / ((FCL_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void CollisionChecker::EulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c,
                                     Matrix3f& R) {
  FCL_REAL c1 = cos(a);
  FCL_REAL c2 = cos(b);
  FCL_REAL c3 = cos(c);
  FCL_REAL s1 = sin(a);
  FCL_REAL s2 = sin(b);
  FCL_REAL s3 = sin(c);

  R.setValue(c1 * c2, -c2 * s1, s2, c3 * s1 + c1 * s2 * s3,
             c1 * c3 - s1 * s2 * s3, -c2 * s3, s1 * s3 - c1 * c3 * s2,
             c3 * s1 * s2 + c1 * s3, c2 * c3);
}

bool CollisionChecker::Init() {
	std::cout << "collision checker init..." << std::endl;

  auto status = voxelizer_->Init();
	if (!status.ok()) {
    std::cout << "init failure: " << status.message() << std::endl;
    return false;
  }

  voxelizer_->VoxelizeSurface(num_thread_);
  Vec3f lb = voxelizer_->MeshLowerBound();
  Vec3f ub = voxelizer_->MeshUpperBound();
  extents_[0] = lb[0];
  extents_[1] = lb[1];
  extents_[2] = lb[2];
  extents_[3] = ub[0];
  extents_[4] = ub[1];
  extents_[5] = ub[2];
  size2_ = size_ * size_;
  voxels_.reset(new VoxelIndex[voxelizer_->TotalVoxelCompressedSize()],
                ArrayDeleter<VoxelIndex>());
  for (int i = 0; i < voxelizer_->TotalVoxelCompressedSize(); ++i)
    voxels_.get()[i] = voxelizer_->Voxels().get()[i];
  unit_.reset(new Box(voxelizer_->Unit()));
  PreMeshCO();

  std::cout << "done." << std::endl;

  return true;
}

COSP CollisionChecker::GenRandomCO(double ratio) {
  BoxSP box(new Box((voxelizer_->MeshUpperBound() -
                     voxelizer_->MeshLowerBound()) *
                    ratio));
  Transform3f tf;
  GenRandomTransform(extents_, tf);
  COSP boxCO(new CollisionObject(box, tf));
  return boxCO;
}

inline bool CollisionChecker::TestVoxel(const COSP& cube_co) {
  const Vec3f lb = voxelizer_->GetVoxel(cube_co->getAABB().min_);
  const Vec3f ub = voxelizer_->GetVoxel(cube_co->getAABB().max_);
  int lx = std::max(0, (int)lb[0]), ux = std::min(size_ - 1, (int)ub[0]),
      ly = std::max(0, (int)lb[1]), uy = std::min(size_ - 1, (int)ub[1]),
      lz = std::max(0, (int)lb[2]), uz = std::min(size_ - 1, (int)ub[2]);
  VoxelIndex voxel_index, tmp;
  Vec3f vxl_box;
  //	std::cout << "e" << std::endl;
  // std::cout << *lb << ", " << *ub << std::endl;
  for (int x = lx, y, z; x <= ux; ++x) {
    for (y = ly; y <= uy; ++y) {
      for (z = lz; z <= uz; ++z) {
        voxel_index = x * size2_ + y * size_ + z;
        tmp = (voxels_.get())[voxel_index / kBatchSize];
        if (!GETBIT(tmp, voxel_index)) continue;
        vxl_box.setValue(x, y, z);
        // Transform3f tf(*voxelizer_->GetLoc(vxl_box));
        Transform3f tf(voxelizer_->GetLoc(vxl_box) +
                       voxelizer_->HalfUnit());
        // std::cout << *voxelizer_->GetLoc(vxl_box) << std::endl;
        CollisionRequest request;
        CollisionResult result;
        COUP box_co(new CollisionObject(unit_, tf));
        if (fcl::collide(box_co.get(), cube_co.get(), request, result)) {
          return true;
        }
      }
    }
  }
  return false;
}

inline bool CollisionChecker::TestMesh(const COSP& cube_co) {
  CollisionRequest request;
  CollisionResult result;
  if (fcl::collide(mesh_co_.get(), cube_co.get(), request, result)) {
    return true;
  }
  return false;
}

void CollisionChecker::Test(int num_cases, double ratio) {
  Timer timer;
  double t1 = 0, t2 = 0;
  int tp = 0, fp = 0, fn = 0, tn = 0;
  for (int i = 0; i < num_cases; ++i) {
    COSP cube_co = GenRandomCO(ratio);
    cube_co->computeAABB();
    //		std::cout << cube_co->getAABB().min_ << std::endl;
    timer.Restart();
    bool res1 = TestVoxel(cube_co);
    timer.Stop();
    t1 += timer.TimeInS();
    timer.Restart();
    bool res2 = TestMesh(cube_co);
    timer.Stop();
    t2 += timer.TimeInS();
    if (res1 && !res2) {
      fp++;
    }
    if (!res1 && res2) {
      fn++;
    }
    if (res1 && res2) {
      tp++;
    }
    if (!res1 && !res2) {
      tn++;
    }
  }
  std::cout << "---------- ratio = " << ratio << " ----------" << std::endl;
  std::cout << "accuracy = \t" << 100.0 * (tp + tn) / (num_cases) << "\%" << std::endl;
  // std::cout << "# = \t" << tp << ", " << tn << "\%" << std::endl;
  std::cout << "voxel consumes \t" << t1 / num_cases << " s" << std::endl;
  std::cout << "mesh consumes \t" << t2 / num_cases << " s" << std::endl;
}

void CollisionChecker::PreMeshCO() {
  vertices_.clear();
  triangles_.clear();
  V3SP tmpVertices = voxelizer_->Vertices();
  for (int i = 0; i < voxelizer_->VerticesSize(); ++i) {
    vertices_.push_back((tmpVertices.get())[i]);
  }
  V3SP tmpFaces = voxelizer_->Faces();
  for (int i = 0; i < voxelizer_->FacesSize(); ++i) {
    Vec3f& tmp = (tmpFaces.get())[i];
    triangles_.push_back(Triangle(tmp[0], tmp[1], tmp[2]));
  }
  ModelSP model(new Model());
  model->beginModel(triangles_.size(), vertices_.size());
  model->addSubModel(vertices_, triangles_);
  model->endModel();
  mesh_co_.reset(new CollisionObject(model));
  mesh_co_->computeAABB();
}

void CollisionChecker::GenRandomTransform(FCL_REAL extents[6],
                                          Transform3f& transform) {
  FCL_REAL x = RandInterval(extents[0], extents[3]);
  FCL_REAL y = RandInterval(extents[1], extents[4]);
  FCL_REAL z = RandInterval(extents[2], extents[5]);

  const FCL_REAL pi = 3.1415926;
  FCL_REAL a = RandInterval(0, 2 * pi);
  FCL_REAL b = RandInterval(0, 2 * pi);
  FCL_REAL c = RandInterval(0, 2 * pi);

  Matrix3f R;
  EulerToMatrix(a, b, c, R);
  Vec3f T(x, y, z);
  transform.setTransform(R, T);
}

CollisionChecker::~CollisionChecker() {
  // TODO Auto-generated destructor stub
}

}  // namespace collision_checker

int run_benchmark () {
  const std::string inputFile[] = {"../../data/kawada-hironx.stl", "../../data/racecar.stl",
                        "../../data/bike.stl"};
  const double ratios[] = {0.005, 0.01, 0.02, 0.04, 0.08};
  const int gridSize[] = {128, 256, 512};
  int testCases = 1000;
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 3; ++i) {
      std::cout << "==================================" << std::endl;
      std::cout << "Intput file : " << inputFile[i] << std::endl;
      std::cout << "Grid size : " << gridSize[k] << std::endl;
      std::cout << "==================================" << std::endl;
      collision_checker::CollisionChecker checker(gridSize[k], 4, inputFile[i]);
      if (!checker.Init()) {
        std::cout << "Checker init error." << std::endl;
        return 1;
      }
      for (int j = 0; j < 5; ++j) {
        checker.Test(testCases, ratios[j]);
      }
    }
  }
  return 0;
}

int run_single() {
  const std::string in_file = "../../data/kawada-hironx.stl";
   collision_checker::CollisionChecker checker(256, 4, in_file);
  if (!checker.Init()) {
    std::cout << "Checker init error." << std::endl;
    return 1;
  }
  checker.Test(10, 0.08);
  return 0;
}

int main(int argc, char* argv[]) {
  return run_single();
}
