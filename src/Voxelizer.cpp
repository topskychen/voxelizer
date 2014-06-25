/*
 * Voxelizer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "Voxelizer.h"
#include <exception>



Voxelizer::Voxelizer(int size, const string& pFile): _size(size) {
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
		int total_size = size*size*size/BATCH_SIZE;
		if (total_size < 0)
			throw std::runtime_error("Size overflow!");

		/**
		 * Reset voxels.
		 */
		_voxels.reset(new auint[total_size], ArrayDeleter<auint>());
		memset(_voxels.get(), 0, total_size * sizeof(int));

		/**
		 * Store info.
		 */
		_numVertices = mesh->mNumVertices;
		_numFaces = mesh->mNumFaces;
		std::cout << "Faces : " << _numFaces << std::endl;
		std::cout << "Vertices : " << _numVertices << std::endl;
		loadFromMesh(mesh);

		randomPermutation();

//		_identity.setIdentity();
		if (!scene) delete scene;
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		if (!scene) delete scene;
	}
	cout << "Voxelizer setup!" << endl;
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
//	std::cout << (*_vertices)[vIds[0]] << ", " << (*_vertices)[vIds[1]] << ", " << (*_vertices)[vIds[2]] << std::endl;
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
			_lb.reset(new Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
							mesh->mVertices[i].z));
			_ub.reset(new Vec3f(mesh->mVertices[i].x, mesh->mVertices[i].y,
							mesh->mVertices[i].z));
		} else {
			_lb->ubound(_vertices.get()[i]); // bug1
			_ub->lbound(_vertices.get()[i]); // bug2
		}
	}
	//TODO this is to show
	_lb.reset(new Vec3f((*_lb)-Vec3f(0.0001, 0.0001, 0.0001)));
	_ub.reset(new Vec3f((*_ub)+Vec3f(0.0001, 0.0001, 0.0001)));
//	_ub->data[0]*=4;
	_bound.reset(new Vec3f((*_ub - *_lb)));
	_faces.reset(new Vec3f[_numFaces], ArrayDeleter<Vec3f>());
	for (size_t i = 0; i < _numFaces; ++i) {
		_faces.get()[i] = Vec3f(mesh->mFaces[i].mIndices[0],
				mesh->mFaces[i].mIndices[1], mesh->mFaces[i].mIndices[2]);
	}
	Vec3f halfUnit = (*_bound) / ((float) _size*2);
	_halfUnit.reset(new Vec3f(halfUnit));
	cout << "Bounds : " << *_lb << ", " << *_ub << endl;
}


/**
 * Run tasks.
 */
void Voxelizer::voxelizeSurface(const int numThread) {
	 ThreadPool tp(numThread);
	 for (int i = 0; i < _numFaces; ++i) {
		 tp.run(boost::bind(&Voxelizer::runSurfaceTask, this, i));
	 }
	 tp.stop();
}

void Voxelizer::runSurfaceTask(const int triId) {
	tri_p tri = getTri(triId);
	tri->computeLocalAABB();
	const v3_p lb = getVoxel(tri->aabb_local.min_);
	const v3_p ub = getVoxel(tri->aabb_local.max_);

	/**
	 * may optimize with bfs here;
	 */
	int count = 0;
	int esti = ((*ub)[0]-(*lb)[0]+1)*((*ub)[1]-(*lb)[1]+1)*((*ub)[2]-(*lb)[2]+1);
	if (esti < 2500) {
		v3_p vxlBox(new Vec3f(0, 0, 0));
		for (int x = (*lb)[0]; x <= (*ub)[0]; ++x) {
			for (int y = (*lb)[1]; y <= (*ub)[1]; ++y) {
				for (int z = (*lb)[2]; z <= (*ub)[2]; ++z) {
					vxlBox->setValue(x, y, z);
					unsigned int voxelInt = convVoxelToInt(vxlBox);
					unsigned int tmp = (_voxels.get())[voxelInt / BATCH_SIZE].load();
					if (GETBIT(tmp, voxelInt))
						continue;
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

bool myLess(Vec3f a, Vec3f b) {
	return a[0] <= b[0] && a[1] <= b[1] && a[2] <= b[2];
}


inline int Voxelizer::bfsSurface(const tri_p& tri, const v3_p& lb, const v3_p& ub) {
	queue<unsigned int> q;
	hash_set set;
	unsigned int start = convVoxelToInt(getVoxel(tri->a));
	q.push(start);
	set.insert(start);
	v3_p topVoxel(new Vec3f(0, 0, 0));
	int count = 0;
	while (!q.empty()) {
		count++;
		unsigned int topVoxelInt = q.front();
		unsigned int tmp = (_voxels.get())[topVoxelInt/BATCH_SIZE].load();
		q.pop();
		topVoxel = convIntToVoxel(topVoxelInt);
		if (GETBIT(tmp, topVoxelInt) || collide(_halfUnit, getLoc(topVoxel), tri)) {
			if (!GETBIT(tmp, topVoxelInt)) {
				(_voxels.get())[topVoxelInt / BATCH_SIZE] |= (1<<(topVoxelInt % BATCH_SIZE));
			}
			for (int i = 0; i < 6; ++i) {
				Vec3f newVoxel = *topVoxel + D_6[i];
				if (!inRange(newVoxel, lb, ub)) continue;
				unsigned int newVoxelInt = convVoxelToInt(newVoxel);
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
//	ThreadPool tp(numThread);
//	for (int i = 0; i < _numFaces; ++i) {
//		task_p task(new Task(getTri(i)));
//		tp.run(boost::bind(&Voxelizer::runSurfaceTask, this, task));
//	}
//	tp.stop();
}

void Voxelizer::runSolidTask(const v3_p& start) {
}

inline bool Voxelizer::inRange(const Vec3f& vc, const v3_p& lb, const v3_p& ub) {
	return vc[0]>=(*lb)[0] && vc[0]<=(*ub)[0] && vc[1]>=(*lb)[1] && vc[1]<=(*ub)[1] && vc[2]>=(*lb)[2] && vc[2]<=(*ub)[2];
}


inline v3_p Voxelizer::convIntToVoxel(const unsigned int& coord) {
	v3_p voxel(new Vec3f(coord/_size/_size, (coord/_size)%_size, coord%_size));
	return voxel;
}

inline unsigned int Voxelizer::convVoxelToInt(const v3_p& voxel) {
	return (*voxel)[0]*_size*_size + (*voxel)[1]*_size + (*voxel)[2];
}

inline unsigned int Voxelizer::convVoxelToInt(const Vec3f& voxel) {
	return voxel[0]*_size*_size + voxel[1]*_size + voxel[2];
}

/**
 * Write to file, with simple compression
 */
void Voxelizer::write(const string& pFile) {

	ofstream* output = new ofstream(pFile.c_str(), ios::out | ios::binary);

//	  Vector norm_translate = voxels.get_norm_translate();
//	  Float norm_scale = voxels.get_norm_scale();

	Vec3f& norm_translate = (*_lb);
	float norm_scale = (_bound->norm());

	//
	// write header
	//
	*output << "#binvox 1" << endl;
	//  *output << "bbox [-1,1][-1,1][-1,1]" << endl;  // no use for 'bbox'
	//  *output << "dim [" << depth << "," << height << "," << width << "]" << endl;
	//  *output << "type RLE" << endl;
	*output << "dim " << _size << " " << _size << " " << _size << endl;
	cout << "Dim : " << _size << " x " << _size << " x " << _size << endl;
	*output << "translate " << -norm_translate[0] << " " << -norm_translate[2]
			<< " " << -norm_translate[1] << endl;
	cout << "translate " << -norm_translate[0] << " " << -norm_translate[2]
				<< " " << -norm_translate[1] << endl;
	*output << "scale " << norm_scale << endl;
	*output << "data" << endl;

	byte value;
	byte count;
	int index = 0;
	int bytes_written = 0;
	int size = _size * _size * _size;
	int total_ones = 0;

	/**
	 * Compression
	 */
	while (index < size) {

		value = GETBIT(_voxels.get()[index/BATCH_SIZE],index);
		count = 0;
		while ((index < size) && (count < 255)
				&& (value == GETBIT(_voxels.get()[index/BATCH_SIZE],index))) {
			index++;
			count++;
		}
		//    value = 1 - value;
		if (value)
			total_ones += count;

		*output << value << count;  // inverted...
		bytes_written += 2;

	}  // while

	output->close();

	cout << "Wrote " << total_ones << " set voxels out of " << size << ", in "
			<< bytes_written << " bytes" << endl;
}

void Voxelizer::randomPermutation() {

	for (int i = 0, id; i < _numFaces; ++i) {
		id = random(i, _numFaces-1);
		if (i != id) swap((_faces.get())[i], (_faces.get())[id]);
	}
	cout << "Random permutation..." << endl;
}


Voxelizer::~Voxelizer() {
	// TODO Auto-generated destructor stub
}

inline const int& Voxelizer::getSize() const {
	return _size;
}

inline const auint_p& Voxelizer::getVoxels() const {
	return _voxels;
}
