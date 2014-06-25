/*
 * Voxelizer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "Voxelizer.h"
#include <exception>



Voxelizer::Voxelizer(int size, const string& pFile): _size(size) {
	cout << "voxelizer init... " << endl;
	const aiScene* scene;
	try {
		/*
		 * Load scene
		 * */
		Assimp::Importer importer;
		scene = importer.ReadFile(pFile, aiProcessPreset_TargetRealtime_Fast);
		if (!scene) {
			throw std::runtime_error("Scene fails to be loaded!");
		}
		aiMesh* mesh = scene->mMeshes[0];
		_size2 = _size*_size;
		_totalSize = size*size*size/BATCH_SIZE;

		/**
		 * Reset voxels.
		 */
		_voxels.reset(new auint[_totalSize], ArrayDeleter<auint>());
		_voxelsBuffer.reset(new auint[_totalSize], ArrayDeleter<auint>());
		memset(_voxels.get(), 0, _totalSize * sizeof(int));
		memset(_voxelsBuffer.get(), 0, _totalSize * sizeof(int));

		/**
		 * Store info.
		 */
		_numVertices = mesh->mNumVertices;
		_numFaces = mesh->mNumFaces;
		std::cout << "faces : " << _numFaces << std::endl;
		std::cout << "vertices : " << _numVertices << std::endl;
		loadFromMesh(mesh);
//		prepareBoundareis();


//		_identity.setIdentity();
		if (!scene) delete scene;
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		if (!scene) delete scene;
	}
	cout << "done." << endl;
}


/**
 * Given voxel (int x, int y, int z), return loc (float x, float y, float z)
 */
inline v3_p Voxelizer::getLoc(const v3_p& voxel) {
	Vec3f tmp = *_lb + (*_bound) * (*voxel) / (float) _size;
	v3_p loc(new Vec3f(tmp));
	return loc;
}


inline v3_p Voxelizer::getLoc(const Vec3f& voxel) {
	Vec3f tmp = *_lb + (*_bound) * (voxel) / (float) _size;
	v3_p loc(new Vec3f(tmp));
	return loc;
}

/**
 * Given loc (float x, float y, float z), return voxel (int x, int y, int z)
 */
inline v3_p Voxelizer::getVoxel(const Vec3f& loc) {
	Vec3f tmp = (loc - (*_lb)) * (float) _size / (*_bound);
	v3_p voxel(new Vec3f((int)tmp[0], (int)tmp[1], (int)tmp[2]));
	return voxel;
}

/**
 * Given loc (float x, float y, float z), return voxel (int x, int y, int z)
 */
inline v3_p Voxelizer::getVoxel(const v3_p& loc) {
	Vec3f tmp = ((*loc) - (*_lb)) * (float) _size / (*_bound);
	v3_p voxel(new Vec3f((int)tmp[0], (int)tmp[1], (int)tmp[2]));
	return voxel;
}



/**
 *Get the collision object form triangle(face) id;
 */
inline tri_p Voxelizer::getTri(const int triId) {
	const Vec3f& vIds = _faces.get()[triId];
	tri_p tri(new TriangleP(_vertices.get()[(int)vIds[0]], _vertices.get()[(int)vIds[1]], _vertices.get()[(int)vIds[2]]));
	return tri;
}

/**
 * Load info from mesh.
 */
inline void Voxelizer::loadFromMesh(const aiMesh* mesh) {
	_vertices.reset(new Vec3f[_numVertices], ArrayDeleter<Vec3f>());
	Vec3f tmp;
	for (size_t i = 0; i < _numVertices; ++i) {
		_vertices.get()[i] = Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
				mesh->mVertices[i].z);
		if (i == 0) {
			_meshLb.reset(new Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
							mesh->mVertices[i].z));
			_meshUb.reset(new Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
							mesh->mVertices[i].z));
		} else {
			_meshLb->ubound(_vertices.get()[i]); // bug1
			_meshUb->lbound(_vertices.get()[i]); // bug2
		}
	}
	//TODO this is to show
	_meshLb.reset(new Vec3f((*_meshLb)-Vec3f(0.0001, 0.0001, 0.0001)));
	_meshUb.reset(new Vec3f((*_meshUb)+Vec3f(0.0001, 0.0001, 0.0001)));


	/**
	*
	*/
	_minLb = (*_meshLb)[0];
	_minLb = min(_minLb, (float)(*_meshLb)[1]);
	_minLb = min(_minLb, (float)(*_meshLb)[2]);
	_maxUb = (*_meshUb)[0];
	_maxUb = max(_maxUb, (float)(*_meshUb)[1]);
	_maxUb = max(_maxUb, (float)(*_meshUb)[2]);
	_lb.reset(new Vec3f(_minLb, _minLb, _minLb));
	_ub.reset(new Vec3f(_maxUb, _maxUb, _maxUb));
	_bound.reset(new Vec3f((*_ub - *_lb)));


	_faces.reset(new Vec3f[_numFaces], ArrayDeleter<Vec3f>());
	for (size_t i = 0; i < _numFaces; ++i) {
		_faces.get()[i] = Vec3f(mesh->mFaces[i].mIndices[0],
				mesh->mFaces[i].mIndices[1], mesh->mFaces[i].mIndices[2]);
	}
	randomPermutation(_faces, _numFaces);
	Vec3f halfUnit = (*_bound) / ((float) _size*2);
	_halfUnit.reset(new Vec3f(halfUnit));
	_meshVoxLB = getVoxel(_meshLb);
	_meshVoxUB = getVoxel(_meshUb);

	cout << *_meshVoxLB << ", " << *_meshVoxUB << endl;
	cout << "whole space : " << *_lb << ", " << *_ub << endl;
	cout << "mesh bound : " << *_meshLb << ", " << *_meshUb << endl;
}


/**
 * Run tasks.
 */
void Voxelizer::voxelizeSurface(const int numThread) {
	 cout << "surface voxelizing... " << endl;
	 ThreadPool tp(numThread);
	 for (int i = 0; i < _numFaces; ++i) {
		 tp.run(boost::bind(&Voxelizer::runSurfaceTask, this, i));
	 }
	 tp.stop();
//	 for (int i = 0; i < _totalSize; ++i) _voxelsBuffer.get()[i] = _voxels.get()[i];
	 cout << "done." << endl;
}

void Voxelizer::runSurfaceTask(const int triId) {
	tri_p tri = getTri(triId);
	tri->computeLocalAABB();
	const v3_p lb = getVoxel(tri->aabb_local.min_);
	const v3_p ub = getVoxel(tri->aabb_local.max_);
	int lx = (*lb)[0], ux = (*ub)[0], ly = (*lb)[1], uy = (*ub)[1], lz = (*lb)[2], uz = (*ub)[2];

	/**
	 * may optimize with bfs here;
	 */
	int count = 0;
	int esti = ((*ub)[0]-(*lb)[0]+1)*((*ub)[1]-(*lb)[1]+1)*((*ub)[2]-(*lb)[2]+1);
	if (true) {
		v3_p vxlBox(new Vec3f(0, 0, 0));
		unsigned int voxelInt, tmp;
		for (int x = lx, y, z; x <= ux; ++x) {
			for (y = ly; y <= uy; ++y) {
				for (z = lz; z <= ux; ++z) {
					voxelInt = x*_size2+y*_size+z;
					tmp = (_voxels.get())[voxelInt / BATCH_SIZE].load();
					if (GETBIT(tmp, voxelInt))
						continue;
					vxlBox->setValue(x, y, z);
					if (collide(_halfUnit, getLoc(vxlBox), tri)) {
						(_voxels.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
						count++;
					}
				}
			}
		}
		// cout << "1: " << count << "\t" << esti << "\t" << 100.0*count/esti <<"%"<< endl;
	} else {
		count = bfsSurface(tri, lb, ub);
		// cout << "2: " << count << "\t" << esti << "\t" << 100.0*count/esti <<"%"<< endl;
	}

}

inline int Voxelizer::bfsSurface(const tri_p& tri, const v3_p& lb, const v3_p& ub) {
	queue<unsigned int> q;
	hash_set set;
	unsigned int start = convVoxelToInt(getVoxel(tri->a)), topVoxelInt, tmp, newVoxelInt;
	q.push(start);
	set.insert(start);
	v3_p topVoxel;
	int count = 0;
	while (!q.empty()) {
		count++;
		topVoxelInt = q.front();
		q.pop();
		tmp = (_voxels.get())[topVoxelInt/BATCH_SIZE].load();
		topVoxel = convIntToVoxel(topVoxelInt);
		if (GETBIT(tmp, topVoxelInt) || collide(_halfUnit, getLoc(topVoxel), tri)) {
			if (!GETBIT(tmp, topVoxelInt)) {
				(_voxels.get())[topVoxelInt / BATCH_SIZE] |= (1<<(topVoxelInt % BATCH_SIZE));
			}
			for (int i = 0; i < 6; ++i) {
				Vec3f newVoxel = *topVoxel + D_6[i];
				if (!inRange(newVoxel, lb, ub)) continue;
				newVoxelInt = convVoxelToInt(newVoxel);
				if (set.find(newVoxelInt) == set.end()) {
					set.insert(newVoxelInt);
					q.push(newVoxelInt);
				}
			}
		}
	}
	return count;
}

void Voxelizer::voxelizeSolid(int numThread) {
	cout << "solid voxelizing... " << endl;
	prepareBoundareis(numThread);
	ThreadPool tp(numThread);
//	for (int i = 0; i < _numBoundaries; ++i) {
//		tp.run(boost::bind(&Voxelizer::runSolidTask, this, i));
//	}
	tp.stop();
	for (int i = 0; i < _totalSize; ++i) _voxels.get()[i] = _voxelsBuffer.get()[i]^(~0);
	cout << "done." << endl;
}

void Voxelizer::runSolidTask(const int voxelId) {
	unsigned int startInt = convVoxelToInt(_boundaries.get()[voxelId]);
	unsigned int tmp = (_voxels.get())[startInt/BATCH_SIZE].load();
	if (GETBIT(tmp, startInt)) return;
	queue<unsigned int> q;
	q.push(startInt);
	v3_p topVoxel(new Vec3f(0, 0, 0));
	while (!q.empty()) {
		unsigned int topVoxelInt = q.front();
		tmp = (_voxels.get())[topVoxelInt/BATCH_SIZE].load();
		q.pop();
		topVoxel = convIntToVoxel(topVoxelInt);
		if (!GETBIT(tmp, topVoxelInt)) {
			(_voxels.get())[topVoxelInt / BATCH_SIZE] |= (1<<(topVoxelInt % BATCH_SIZE));
			for (int i = 0; i < 6; i++) {
				Vec3f newVoxel = *topVoxel + D_6[i];
				if (!inRange(newVoxel, _meshVoxLB, _meshVoxUB)) continue;
				unsigned int newVoxelInt = convVoxelToInt(newVoxel);
				tmp = (_voxels.get())[newVoxelInt/BATCH_SIZE].load();
				if(!GETBIT(tmp, newVoxelInt)) q.push(newVoxelInt);
			}
		}

	}
}

inline bool Voxelizer::inRange(const Vec3f& vc, const v3_p& lb, const v3_p& ub) {
	return vc[0]>=(*lb)[0] && vc[0]<=(*ub)[0] && vc[1]>=(*lb)[1] && vc[1]<=(*ub)[1] && vc[2]>=(*lb)[2] && vc[2]<=(*ub)[2];
}


inline v3_p Voxelizer::convIntToVoxel(const unsigned int& coord) {
	v3_p voxel(new Vec3f(coord/_size2, (coord/_size)%_size, coord%_size));
	return voxel;
}

inline unsigned int Voxelizer::convVoxelToInt(const v3_p& voxel) {
	return (*voxel)[0]*_size2 + (*voxel)[1]*_size + (*voxel)[2];
}

inline unsigned int Voxelizer::convVoxelToInt(const Vec3f& voxel) {
	return voxel[0]*_size2 + voxel[1]*_size + voxel[2];
}


/**
 * Write to file, with simple compression
 */
void Voxelizer::write(const string& pFile) {

	cout << "writing voxels to file..." << endl;

	v3_p vxlBox(new Vec3f(0, 0, 0));
	int lx = (*_meshVoxLB)[0], ux = (*_meshVoxUB)[0], ly = (*_meshVoxLB)[1], uy = (*_meshVoxUB)[1], lz = (*_meshVoxLB)[2], uz = (*_meshVoxUB)[2];
	int bx = ux-lx+1, by = uy-ly+1, bz = uz-lz+1;

	ofstream* output = new ofstream(pFile.c_str(), ios::out | ios::binary);

//	  Vector norm_translate = voxels.get_norm_translate();
//	  Float norm_scale = voxels.get_norm_scale();

	Vec3f& norm_translate = (*_meshLb);
	float norm_scale = ((*_meshUb-*_meshLb).norm());

	//
	// write header
	//
	*output << "#binvox 1" << endl;
	//  *output << "bbox [-1,1][-1,1][-1,1]" << endl;  // no use for 'bbox'
	//  *output << "dim [" << depth << "," << height << "," << width << "]" << endl;
	//  *output << "type RLE" << endl;
	*output << "dim " << bx  << " " << by << " " << bz << endl;
	cout << "dim : " << bx << " x " << by << " x " << bz << endl;
	*output << "translate " << -norm_translate[0] << " " << -norm_translate[2]
			<< " " << -norm_translate[1] << endl;
	*output << "scale " << norm_scale << endl;
	*output << "data" << endl;

	byte value;
	byte count;
	int index = 0;
	int bytes_written = 0;
	int total_ones = 0;

	/**
	 * Compression
	 */
	int x = lx, y = ly, z = lz;
	while (x <= ux) {
		index = x*_size2 + y*_size + z;
		value = GETBIT(_voxels.get()[index/BATCH_SIZE],index);
		count = 0;
		while ((x <= ux) && (count < 255) && (value == GETBIT(_voxels.get()[index/BATCH_SIZE],index))) {
			z++;
			if (z > uz) {
				z = lz;
				y++;
				if (y > uy) {
					y = ly;
					x++;
				}
			}
			index = x*_size2 + y*_size + z;
			count++;
		}
		if (value)
			total_ones += count;

		*output << value << count;  // inverted...
		bytes_written += 2;

	}

//	while (index < size) {
//
//		value = GETBIT(_voxels.get()[index/BATCH_SIZE],index);
//		count = 0;
//		while ((index < size) && (count < 255)
//				&& (value == GETBIT(_voxels.get()[index/BATCH_SIZE],index))) {
//			index++;
//			count++;
//		}
//		//    value = 1 - value;
//		if (value)
//			total_ones += count;
//
//		*output << value << count;  // inverted...
//		bytes_written += 2;
//
//	}  // while

	output->close();

	cout << "wrote " << total_ones << " set voxels out of " << bx*by*bz << ", in "
			<< bytes_written << " bytes" << endl;
}

void Voxelizer::randomPermutation(const v3_p& data, int num) {

	for (int i = 0, id; i < num; ++i) {
		id = random(i, num-1);
		if (i != id) swap((data.get())[i], (data.get())[id]);
	}
//	cout << "random permutating... " << endl;
}

inline void Voxelizer::fillYZ(const int x) {
	int ly = (*_meshVoxLB)[1], uy = (*_meshVoxUB)[1], lz = (*_meshVoxLB)[2], uz = (*_meshVoxUB)[2];
	unsigned int voxelInt, tmp;
	for (int y = ly, z; y <= uy; ++y) {
		for (z = lz; z <= uz; ++z) {
			voxelInt = x*_size2 + y*_size + z;
			tmp = (_voxels.get())[voxelInt/BATCH_SIZE].load();
			if (GETBIT(tmp, voxelInt)) break;
			else {
				(_voxelsBuffer.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
			}
		}
		if (z == uz+1) continue;
		for (z = uz; z >= lz; --z) {
			voxelInt = x*_size2 + y*_size + z;
			tmp = (_voxels.get())[voxelInt/BATCH_SIZE].load();
			if (GETBIT(tmp, voxelInt)) break;
			else {
				(_voxelsBuffer.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
			}
		}
	}
}

inline void Voxelizer::fillXZ(const int y) {
	int lx = (*_meshVoxLB)[0], ux = (*_meshVoxUB)[0], lz = (*_meshVoxLB)[2], uz = (*_meshVoxUB)[2];
	unsigned int voxelInt, tmp;
	for (int z = lz, x; z <= uz; ++z) {
		for (x = lx; x <= ux; ++x) {
			voxelInt = x*_size2 + y*_size + z;
			tmp = (_voxels.get())[voxelInt/BATCH_SIZE].load();
			if (GETBIT(tmp, voxelInt)) break;
			else {
				(_voxelsBuffer.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
			}
		}
		if (x == ux+1) continue;
		for (x = ux; x >= lx; --x) {
			voxelInt = x*_size2 + y*_size + z;
			tmp = (_voxels.get())[voxelInt/BATCH_SIZE].load();
			if (GETBIT(tmp, voxelInt)) break;
			else {
				(_voxelsBuffer.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
			}
		}
	}
}

inline void Voxelizer::fillXY(const int z) {
	int ly = (*_meshVoxLB)[1], uy = (*_meshVoxUB)[1], lx = (*_meshVoxLB)[0], ux = (*_meshVoxUB)[0];
	unsigned int voxelInt, tmp;
	for (int x = lx, y; x <= ux; ++x) {
		for (y = ly; y <= uy; ++y) {
			voxelInt = x*_size2 + y*_size + z;
			tmp = (_voxels.get())[voxelInt/BATCH_SIZE].load();
			if (GETBIT(tmp, voxelInt)) break;
			else {
				(_voxelsBuffer.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
			}
		}
		if (y == uy+1) continue;
		for (y = uy; y >= ly; --y) {
			voxelInt = x*_size2 + y*_size + z;
			tmp = (_voxels.get())[voxelInt/BATCH_SIZE].load();
			if (GETBIT(tmp, voxelInt)) break;
			else {
				(_voxelsBuffer.get())[voxelInt / BATCH_SIZE] |= (1<< (voxelInt % BATCH_SIZE));
			}
		}
	}
}

inline void Voxelizer::prepareBoundareis(size_t numThread) {
	ThreadPool tp(numThread);
	int lx = (*_meshVoxLB)[0], ux = (*_meshVoxUB)[0], ly = (*_meshVoxLB)[1], uy = (*_meshVoxUB)[1], lz = (*_meshVoxLB)[2], uz = (*_meshVoxUB)[2];
	for (int x = lx; x <= ux; ++x) {
		tp.run(boost::bind(&Voxelizer::fillYZ, this, x));
	}
	for (int z = lz; z <= uz; ++z) {
		tp.run(boost::bind(&Voxelizer::fillXY, this, z));
	}
	for (int y = ly; y <= uy; ++y) {
		tp.run(boost::bind(&Voxelizer::fillXZ, this, y));
	}
	tp.stop();
//	randomPermutation(_boundaries, _numBoundaries);
}

Voxelizer::~Voxelizer() {
	// TODO Auto-generated destructor stub
}

inline const auint_p& Voxelizer::getVoxels() const {
	return _voxels;
}
