/*
 * Commons.h
 *
 *  Created on: 23 Jun, 2014
 *      Author: chenqian
 */

#ifndef COMMONS_H_
#define COMMONS_H_

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include "TriBox.h"

#include <boost/unordered_set.hpp>
#include <ctime>
#include <cstdlib>
using namespace fcl;

typedef boost::shared_ptr<TriangleP> tri_p;
typedef boost::shared_ptr<Box> box_p;
typedef boost::shared_ptr<Vec3f> v3_p;

typedef boost::atomic_uint auint;
typedef boost::shared_ptr<boost::atomic_uint> auint_p;

typedef boost::unordered_set<unsigned int> hash_set;
typedef boost::shared_ptr<unsigned int> uint_p;

typedef unsigned char byte;

bool collide(const v3_p& size, const v3_p& boxAA, const tri_p& tri);
inline void fill(const Vec3f& vc, float ft[3]);

int random(const int l, const int r);

const Vec3f D_8[] = {
	Vec3f(1, 1, 1),
	Vec3f(1, 1, -1),
	Vec3f(1, -1, 1),
	Vec3f(1, -1, -1),
	Vec3f(-1, 1, 1),
	Vec3f(-1, 1, -1),
	Vec3f(-1, -1, 1),
	Vec3f(-1, -1, -1)
};

const Vec3f D_6[] = {
	Vec3f(0, 0, -1),
	Vec3f(0, 0, 1),
	Vec3f(-1, 0, 0),
	Vec3f(1, 0, 0),
	Vec3f(0, -1, 0),
	Vec3f(0, 1, 0),
};

const Vec3f D_26[] = {
	Vec3f(-1, -1,-1),
	Vec3f(-1, -1,0),
	Vec3f(-1, -1,1),
	Vec3f(-1, 0,-1),
	Vec3f(-1, 0,0),
	Vec3f(-1, 0,1),
	Vec3f(-1, 1,-1),
	Vec3f(-1, 1,0),
	Vec3f(-1, 1,1),
	Vec3f(0, -1,-1),
	Vec3f(0, -1,0),
	Vec3f(0, -1,1),
	Vec3f(0, 0,-1),
	Vec3f(0, 0,1),
	Vec3f(0, 1,-1),
	Vec3f(0, 1,0),
	Vec3f(0, 1,1),
	Vec3f(1, -1,-1),
	Vec3f(1, -1,0),
	Vec3f(1, -1,1),
	Vec3f(1, 0,-1),
	Vec3f(1, 0,0),
	Vec3f(1, 0,1),
	Vec3f(1, 1,-1),
	Vec3f(1, 1,0),
	Vec3f(1, 1,1)
};
//class Commons {
//public:
//	Commons();
//	virtual ~Commons();
//};

#endif /* COMMONS_H_ */
