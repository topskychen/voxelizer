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
#include <queue>
#include <fstream>

using namespace std;

const int BATCH_SIZE = 32;

#define GETBIT(x,i) ((x>>(i%BATCH_SIZE))&1)

class Voxelizer;
typedef boost::shared_ptr<Voxelizer> vxlizer_p;


template<typename T>
struct ArrayDeleter {
	void operator ()(T const * p) {
		delete[] p;
//		std::cout << "Array is deleted" << std::endl;
	}
};

class Voxelizer {
	v3_p _lb, _ub, _bound;
	v3_p _halfUnit;

	v3_p _faces;
	int _numFaces;

	v3_p _vertices;
	int _numVertices;

	int _size;
	auint_p _voxels;

//	fcl::Transform3f _identity;

	inline void loadFromMesh(const aiMesh* mesh);
public:
	const int& getSize() const;
	inline const auint_p& getVoxels() const;
	inline v3_p getVoxel(const Vec3f& loc);
	inline v3_p getVoxel(const v3_p& loc);
	inline v3_p getLoc(const v3_p& voxel);
	inline v3_p getLoc(const Vec3f& voxel);
	inline tri_p getTri(const int triId);
	inline bool inRange(const Vec3f& vc, const v3_p& lb, const v3_p& ub);
	inline v3_p convIntToVoxel(const unsigned int& coord);
	inline unsigned int convVoxelToInt(const v3_p& voxel);
	inline unsigned int convVoxelToInt(const Vec3f& voxel);
	inline int bfsSurface(const tri_p& tri, const v3_p& lb, const v3_p& ub);
	void randomPermutation();
	void voxelizeSurface(int numThread=1);
	void runSurfaceTask(const int triId);
	void voxelizeSolid(int numThread=1);
	void runSolidTask(const v3_p& start);
	void collectResult();
	void write(const string& pFile);
	Voxelizer(int size, const string& pFile);
	virtual ~Voxelizer();
};




#endif /* VOXELIZER_H_ */
