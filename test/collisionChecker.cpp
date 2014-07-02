/*
 * collisionChecker.cpp
 *
 *  Created on: 30 Jun, 2014
 *      Author: chenqian
 */

#include "collisionChecker.h"
using namespace std;


FCL_REAL CollisionChecker::_RandInterval(FCL_REAL rmin, FCL_REAL rmax) {
	FCL_REAL t = rand() / ((FCL_REAL)RAND_MAX + 1);
	return (t * (rmax - rmin) + rmin);
}

void CollisionChecker::_EulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R) {
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

CollisionChecker::CollisionChecker(int size, int numThread, const string& pFile) {
	cout << "collision checker init..." << endl;
	_voxelizer.reset(new Voxelizer(size, pFile, false));
	_voxelizer->VoxelizeSurface(numThread);
	// _voxelizer->VoxelizeSolid(numThread);
//	v3_p lb = _voxelizer->GetLowerBound();
//	v3_p ub = _voxelizer->GetUpperBound();
	v3_p lb = _voxelizer->GetMeshLowerBound();
	v3_p ub = _voxelizer->GetMeshUpperBound();
	_extents[0] = (*lb)[0];
	_extents[1] = (*lb)[1];
	_extents[2] = (*lb)[2];
	_extents[3] = (*ub)[0];
	_extents[4] = (*ub)[1];
	_extents[5] = (*ub)[2];
	_size = size;
	_size2 = _size*_size;
	_voxels.reset(new unsigned int[_voxelizer->GetTotalSize()], ArrayDeleter<unsigned int>());
	for (int i = 0; i < _voxelizer->GetTotalSize(); ++i) _voxels.get()[i] = _voxelizer->GetVoxels().get()[i];
	_unit.reset(new Box((*_voxelizer->GetHalfUnit())*2));
	_PreMeshCO();

	cout << "done." << endl;
}

co_p CollisionChecker::_GenRandomCO(double ratio) {
	box_p box(new Box((*(_voxelizer->GetMeshUpperBound())-*(_voxelizer->GetMeshLowerBound()))*ratio));
	Transform3f tf;	
	_GenRandomTransform(_extents, tf);
	co_p boxCO(new CollisionObject(box, tf));
	return boxCO;
}

inline bool CollisionChecker::TestVoxel(const co_p& cubeCO) {
	const v3_p lb = _voxelizer->GetVoxel(cubeCO->getAABB().min_);
	const v3_p ub = _voxelizer->GetVoxel(cubeCO->getAABB().max_);
	int lx = max(0, (int)(*lb)[0]), ux = min(_size-1, (int)(*ub)[0]), ly = max(0, (int)(*lb)[1]), uy = min(_size-1, (int)(*ub)[1]), lz = max(0, (int)(*lb)[2]), uz = min(_size-1, (int)(*ub)[2]);
	unsigned int voxelInt, tmp;
	v3_p vxlBox(new Vec3f(0, 0, 0));
//	cout << "e" << endl;
	// cout << *lb << ", " << *ub << endl;
	for (int x = lx, y, z; x <= ux; ++x) {
		for (y = ly; y <= uy; ++y) {
			for (z = lz; z <= uz; ++z) {
				voxelInt = x * _size2 + y * _size + z;
				tmp = (_voxels.get())[voxelInt / BATCH_SIZE];
				if (!GETBIT(tmp, voxelInt))
					continue;
				vxlBox->setValue(x, y, z);
				// Transform3f tf(*_voxelizer->GetLoc(vxlBox));
				Transform3f tf(*_voxelizer->GetLoc(vxlBox)+*_voxelizer->GetHalfUnit());
				// cout << *_voxelizer->GetLoc(vxlBox) << endl;
				CollisionRequest request;
				CollisionResult result;
				co_p boxCO(new CollisionObject(_unit, tf));
				if (fcl::collide(boxCO.get(), cubeCO.get(), request, result)) {
					return true;
				}
			}
		}
	}
	return false;
}

inline bool CollisionChecker::TestMesh(const co_p& cubeCO) {
	CollisionRequest request;
	CollisionResult result;
	if (fcl::collide(_meshCO.get(), cubeCO.get(), request, result)) {
		return true;
	}
	return false;

}

void CollisionChecker::Test(int numCases, double ratio) {
	Timer timer;
	double t1 = 0, t2 = 0;
	int tp = 0, fp = 0, fn = 0, tn = 0;
	for (int i = 0; i < numCases; ++i) {
		co_p cubeCO = _GenRandomCO(ratio);
		cubeCO->computeAABB();
//		cout << cubeCO->getAABB().min_ << endl;
		timer.Restart();
		bool res1 = TestVoxel(cubeCO);
		timer.Stop();
		t1 += timer.TimeInS();
		timer.Restart();
		bool res2 = TestMesh(cubeCO);
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
	cout << "accuracy = \t" << 100.0 * (tp+tn)/(numCases) << "\%" << endl;
	// cout << "# = \t" << tp << ", " << tn << "\%" << endl;
	cout << "voxel consumes \t" << t1/numCases  << " s" << endl;
	cout << "mesh consumes \t" << t2/numCases << " s" << endl;
}


void CollisionChecker::_PreMeshCO() {
	_vertices.clear();
	_triangles.clear();
	v3_p tmpVertices = _voxelizer->GetVertices();
	for (int i = 0; i < _voxelizer->GetVerticesSize(); ++i) {
		_vertices.push_back((tmpVertices.get())[i]);
	}
	v3_p tmpFaces = _voxelizer->GetFaces();
	for (int i = 0; i < _voxelizer->GetFacesSize(); ++i) {
		Vec3f& tmp = (tmpFaces.get())[i];
		_triangles.push_back(Triangle(tmp[0], tmp[1], tmp[2]));
 	}
	model_p model(new Model());
	model->beginModel(_triangles.size(), _vertices.size());
	model->addSubModel(_vertices, _triangles);
	model->endModel();
	_meshCO.reset(new CollisionObject(model));
	_meshCO->computeAABB();
}

void CollisionChecker::_GenRandomTransform(FCL_REAL extents[6], Transform3f& transform) {
	FCL_REAL x = _RandInterval(extents[0], extents[3]);
	FCL_REAL y = _RandInterval(extents[1], extents[4]);
	FCL_REAL z = _RandInterval(extents[2], extents[5]);

	const FCL_REAL pi = 3.1415926;
	FCL_REAL a = _RandInterval(0, 2 * pi);
	FCL_REAL b = _RandInterval(0, 2 * pi);
	FCL_REAL c = _RandInterval(0, 2 * pi);

	Matrix3f R;
	_EulerToMatrix(a, b, c, R);
	Vec3f T(x, y, z);
	transform.setTransform(R, T);
}


CollisionChecker::~CollisionChecker() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char* argv[]) {
	string inputFile[] = {"../data/kawada-hironx.stl", "../data/racecar.stl", "../data/bike.stl"};
	double ratios[] = {0.005, 0.01, 0.02, 0.04, 0.08};
	int gridSize[] = {128, 256};
	int testCases = 1000;
	for (int i = 0; i < 3; ++i) {
		for (int k = 0; k < 2; ++k) {
			cout << "==================================" << endl;
			cout << "Intput file : " << inputFile[i] << endl;
			cout << "Grid size : " << gridSize[k] << endl;
			cout << "==================================" << endl;
			CollisionChecker checker(gridSize[k], 4, inputFile[i]);
			for (int j = 0; j < 5; ++j) {
				checker.Test(testCases, ratios[j]);		
			}	
		}
			
	}
}


