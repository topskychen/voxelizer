/*
 * Commons.h
 *
 *  Created on: 23 Jun, 2014
 *      Author: chenqian
 */

#ifndef COMMONS_H_
#define COMMONS_H_

#include <assimp/postprocess.h>  // Post processing flags
#include <assimp/scene.h>        // Output data structure
#include <fcl/collision.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

#include <assimp/Importer.hpp>  // C++ importer interface
#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>
#include <cstdlib>
#include <ctime>

#include "tri_box.h"
using namespace fcl;

namespace voxelizer {

// we use unsigned int64 as the index
typedef uint64_t VoxelIndex;
typedef uint32_t VoxelFlags;
// we batch with 64 bit.
const int kBatchSize = 64;

typedef boost::shared_ptr<TriangleP> TriSP;
typedef boost::shared_ptr<Box> BoxSP;
typedef boost::shared_ptr<Vec3f> V3SP;
typedef std::unique_ptr<Vec3f> V3UP;

typedef std::atomic<VoxelIndex> AVI;
typedef boost::shared_ptr<std::atomic<VoxelIndex>> AVISP;

typedef boost::unordered_set<VoxelIndex> HashSet;
typedef boost::shared_ptr<VoxelIndex> VISP;

typedef unsigned char Byte;

template <typename T>
struct ArrayDeleter {
  void operator()(T const* p) { delete[] p; }
};

bool Collide(const Vec3f& size, const Vec3f& boxAA, const TriangleP& tri);
inline void Fill(const Vec3f& vc, float ft[3]);

int Random(const int l, const int r);

const Vec3f D_8[] = {Vec3f(1, 1, 1),   Vec3f(1, 1, -1),  Vec3f(1, -1, 1),
                     Vec3f(1, -1, -1), Vec3f(-1, 1, 1),  Vec3f(-1, 1, -1),
                     Vec3f(-1, -1, 1), Vec3f(-1, -1, -1)};

const int DI_4[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

const Vec3f D_6[] = {
    Vec3f(0, 0, -1), Vec3f(0, 0, 1),  Vec3f(-1, 0, 0),
    Vec3f(1, 0, 0),  Vec3f(0, -1, 0), Vec3f(0, 1, 0),
};

const Vec3f D_26[] = {
    Vec3f(-1, -1, -1), Vec3f(-1, -1, 0), Vec3f(-1, -1, 1), Vec3f(-1, 0, -1),
    Vec3f(-1, 0, 0),   Vec3f(-1, 0, 1),  Vec3f(-1, 1, -1), Vec3f(-1, 1, 0),
    Vec3f(-1, 1, 1),   Vec3f(0, -1, -1), Vec3f(0, -1, 0),  Vec3f(0, -1, 1),
    Vec3f(0, 0, -1),   Vec3f(0, 0, 1),   Vec3f(0, 1, -1),  Vec3f(0, 1, 0),
    Vec3f(0, 1, 1),    Vec3f(1, -1, -1), Vec3f(1, -1, 0),  Vec3f(1, -1, 1),
    Vec3f(1, 0, -1),   Vec3f(1, 0, 0),   Vec3f(1, 0, 1),   Vec3f(1, 1, -1),
    Vec3f(1, 1, 0),    Vec3f(1, 1, 1)};

bool ToVector3Int(const std::vector<std::string>& vs, std::vector<int>& vi);
bool ToVector3Float(const std::vector<std::string>& vs, std::vector<float>& vf);

}  // namespace voxelizer

#endif /* COMMONS_H_ */
