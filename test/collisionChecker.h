/*
 * collisionChecker.h
 *
 *  Created on: 30 Jun, 2014
 *      Author: chenqian
 */

#ifndef COLLISIONCHECKER_H_
#define COLLISIONCHECKER_H_

#include "voxelizer.h"
#include "commons.h"
#include "time.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/math/transform.h"
#include <cstdio>
#include <cstddef>
#include <cmath>


typedef boost::shared_ptr<Voxelizer> vox_p;
typedef boost::shared_ptr<CollisionObject> co_p;
typedef fcl::BVHModel<OBBRSS> Model;
typedef boost::shared_ptr<Model> model_p;
typedef boost::shared_ptr<unsigned int> uint_p;

class CollisionChecker {

	void _EulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R);
	co_p _GenRandomCO(double ratio);
	void _PreMeshCO();
	FCL_REAL _RandInterval(FCL_REAL rmin, FCL_REAL rmax);
	FCL_REAL _extents[6];
	void _GenRandomTransform(FCL_REAL extents[6], Transform3f& transform);
	vox_p _voxelizer;
	co_p _meshCO;
	vector<Vec3f> _vertices;
	vector<Triangle> _triangles;
	int _size, _size2;
	uint_p _voxels;
	box_p _unit;

public:
	inline bool TestVoxel(const co_p& cubeCO);
	inline bool TestMesh(const co_p& cubeCO);
	void Test(int numCases, double ratio);
	CollisionChecker(int size, int numThread, const string& pFile);
	virtual ~CollisionChecker();
};

#endif /* COLLISIONCHECKER_H_ */
