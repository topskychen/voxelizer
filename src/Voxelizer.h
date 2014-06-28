/*
 * Voxelizer.h
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#ifndef VOXELIZER_H_
#define VOXELIZER_H_

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include "Commons.h"
#include "ThreadPool.h"
#include "Timer.h"
#include <queue>
#include <fstream>

using namespace std;

const int BATCH_SIZE = 32;

#define GETBIT(x,i) ((x>>(i%BATCH_SIZE))&1)

template<typename T>
struct ArrayDeleter {
	void operator ()(T const * p) {
		delete[] p;
	}
};

class Voxelizer {

	bool _isInit;

	v3_p _meshLb, _meshUb; // location
	v3_p _meshVoxLB, _meshVoxUB; // voxels of location

	float _minLb, _maxUb;
	v3_p _lb, _ub, _bound; // lowerBound and upperBound of the whole space
	v3_p _halfUnit; // half size of the unit

	v3_p _faces;
	int _numFaces;

	v3_p _vertices;
	int _numVertices;

	auint_p _voxelsBuffer;
	auint_p _voxels;

	unsigned int _size, _totalSize, _size2; // size_2 = size*size

	inline void _loadFromMesh(const aiMesh* mesh);
	inline void _runSolidTask(size_t numThread=1);
	inline void _runSolidTask2(size_t numThread=1);
	inline void _runSurfaceTask(const int triId);
	inline void _fillYZ(const int x);
	inline void _fillXZ(const int y);
	inline void _fillXY(const int z);
	inline void _fillYZ2(const int x);
	inline void _fillXZ2(const int y);
	inline void _fillXY2(const int z);
	inline bool _inRange(const Vec3f& vc, const v3_p& lb, const v3_p& ub);
	inline bool _inRange(const int& x, const int& y, const int& z, const int& lx, const int& ly, const int& lz, const int& ux, const int& uy, const int& uz);
	inline v3_p _getVoxel(const Vec3f& loc);
	inline v3_p _getVoxel(const v3_p& loc);
	inline v3_p _getLoc(const v3_p& voxel);
	inline v3_p _getLoc(const Vec3f& voxel);
	inline tri_p _getTri(const int triId);
	inline v3_p _convIntToVoxel(const unsigned int& coord);
	inline unsigned int _convVoxelToInt(const v3_p& voxel);
	inline unsigned int _convVoxelToInt(const Vec3f& voxel);
	inline int _bfsSurface(const tri_p& tri, const v3_p& lb, const v3_p& ub);
	inline void _randomPermutation(const v3_p& data, int num);
	inline void _bfsSolid(const unsigned int voxelId);
public:

	void voxelizeSurface(int numThread=1);
	void voxelizeSolid(int numThread=1);
	void write(const string& pFile);
	void writeSimple(const string& pFile);
	void writeForView(const string& pFile);
	Voxelizer(int size, const string& pFile);
	virtual ~Voxelizer();
};

#endif /* VOXELIZER_H_ */
