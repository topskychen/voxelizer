/*
 * collisionChecker.cpp
 *
 *  Created on: 30 Jun, 2014
 *      Author: chenqian
 */

#include "collisionChecker.h"
using namespace std;

using voxelizer::ArrayDeleter;
using voxelizer::BoxSP;
using voxelizer::kBatchSize;
using voxelizer::Timer;
using voxelizer::V3SP;
using voxelizer::Voxelizer;

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

CollisionChecker::CollisionChecker(int size, int num_thread,
                                   const string& p_file) {
  cout << "collision checker init..." << endl;
  voxelizer_.reset(new Voxelizer(size, p_file, false));
  voxelizer_->VoxelizeSurface(num_thread);
  // voxelizer_->VoxelizeSolid(num_thread);
  //	V3SP lb = voxelizer_->GetLowerBound();
  //	V3SP ub = voxelizer_->GetUpperBound();
  V3SP lb = voxelizer_->GetMeshLowerBound();
  V3SP ub = voxelizer_->GetMeshUpperBound();
  extents_[0] = (*lb)[0];
  extents_[1] = (*lb)[1];
  extents_[2] = (*lb)[2];
  extents_[3] = (*ub)[0];
  extents_[4] = (*ub)[1];
  extents_[5] = (*ub)[2];
  size_ = size;
  size2_ = size_ * size_;
  voxels_.reset(new unsigned int[voxelizer_->GetTotalSize()],
                ArrayDeleter<unsigned int>());
  for (int i = 0; i < voxelizer_->GetTotalSize(); ++i)
    voxels_.get()[i] = voxelizer_->GetVoxels().get()[i];
  unit_.reset(new Box((*voxelizer_->GetHalfUnit()) * 2));
  PreMeshCO();

  cout << "done." << endl;
}

co_p CollisionChecker::GenRandomCO(double ratio) {
  BoxSP box(new Box((*(voxelizer_->GetMeshUpperBound()) -
                     *(voxelizer_->GetMeshLowerBound())) *
                    ratio));
  Transform3f tf;
  GenRandomTransform(extents_, tf);
  co_p boxCO(new CollisionObject(box, tf));
  return boxCO;
}

inline bool CollisionChecker::TestVoxel(const co_p& cube_co) {
  const V3SP lb = voxelizer_->GetVoxel(cube_co->getAABB().min_);
  const V3SP ub = voxelizer_->GetVoxel(cube_co->getAABB().max_);
  int lx = max(0, (int)(*lb)[0]), ux = min(size_ - 1, (int)(*ub)[0]),
      ly = max(0, (int)(*lb)[1]), uy = min(size_ - 1, (int)(*ub)[1]),
      lz = max(0, (int)(*lb)[2]), uz = min(size_ - 1, (int)(*ub)[2]);
  unsigned int voxelInt, tmp;
  V3SP vxlBox(new Vec3f(0, 0, 0));
  //	cout << "e" << endl;
  // cout << *lb << ", " << *ub << endl;
  for (int x = lx, y, z; x <= ux; ++x) {
    for (y = ly; y <= uy; ++y) {
      for (z = lz; z <= uz; ++z) {
        voxelInt = x * size2_ + y * size_ + z;
        tmp = (voxels_.get())[voxelInt / kBatchSize];
        if (!GETBIT(tmp, voxelInt)) continue;
        vxlBox->setValue(x, y, z);
        // Transform3f tf(*voxelizer_->GetLoc(vxlBox));
        Transform3f tf(*voxelizer_->GetLoc(vxlBox) +
                       *voxelizer_->GetHalfUnit());
        // cout << *voxelizer_->GetLoc(vxlBox) << endl;
        CollisionRequest request;
        CollisionResult result;
        co_p boxCO(new CollisionObject(unit_, tf));
        if (fcl::collide(boxCO.get(), cube_co.get(), request, result)) {
          return true;
        }
      }
    }
  }
  return false;
}

inline bool CollisionChecker::TestMesh(const co_p& cube_co) {
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
    co_p cube_co = GenRandomCO(ratio);
    cube_co->computeAABB();
    //		cout << cube_co->getAABB().min_ << endl;
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
  cout << "---------- ratio = " << ratio << " ----------" << endl;
  cout << "accuracy = \t" << 100.0 * (tp + tn) / (num_cases) << "\%" << endl;
  // cout << "# = \t" << tp << ", " << tn << "\%" << endl;
  cout << "voxel consumes \t" << t1 / num_cases << " s" << endl;
  cout << "mesh consumes \t" << t2 / num_cases << " s" << endl;
}

void CollisionChecker::PreMeshCO() {
  vertices_.clear();
  triangles_.clear();
  V3SP tmpVertices = voxelizer_->GetVertices();
  for (int i = 0; i < voxelizer_->GetVerticesSize(); ++i) {
    vertices_.push_back((tmpVertices.get())[i]);
  }
  V3SP tmpFaces = voxelizer_->GetFaces();
  for (int i = 0; i < voxelizer_->GetFacesSize(); ++i) {
    Vec3f& tmp = (tmpFaces.get())[i];
    triangles_.push_back(Triangle(tmp[0], tmp[1], tmp[2]));
  }
  model_p model(new Model());
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

int main(int argc, char* argv[]) {
  string inputFile[] = {"../data/kawada-hironx.stl", "../data/racecar.stl",
                        "../data/bike.stl"};
  double ratios[] = {0.005, 0.01, 0.02, 0.04, 0.08};
  int gridSize[] = {128, 256, 512};
  int testCases = 1000;
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 3; ++i) {
      cout << "==================================" << endl;
      cout << "Intput file : " << inputFile[i] << endl;
      cout << "Grid size : " << gridSize[k] << endl;
      cout << "==================================" << endl;
      collision_checker::CollisionChecker checker(gridSize[k], 4, inputFile[i]);
      for (int j = 0; j < 5; ++j) {
        checker.Test(testCases, ratios[j]);
      }
    }
  }
}
