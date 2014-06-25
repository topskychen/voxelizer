//============================================================================
// Name        : mujin.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include "Voxelizer.h"
#include "Timer.h"
#include "ThreadPool.h"
#include <iostream>

using namespace std;

typedef boost::shared_ptr<aiScene> scene_p;
typedef boost::shared_ptr<aiMesh> mesh_p;



int main(int args, char* argv[]) {
	Timer timer;
//	string fileName = "/Users/chenqian/workspace/mujin/data/kawada-hironx.stl";
//	string fileName2 = "/Users/chenqian/workspace/mujin/data/test.binvox";
	if (args == 5) {
		int gridSize = atoi(argv[1]);
		int numThread = atoi(argv[2]);
		string inputFile = argv[3];
		string outputFile = argv[4];
		timer.restart();
		Voxelizer voxelizer(gridSize, inputFile);
		voxelizer.voxelizeSurface(numThread);
		timer.stop();
		timer.printTimeInS();
		voxelizer.voxelizeSolid(numThread);
		timer.stop();
		timer.printTimeInS();
		timer.restart();
		voxelizer.write(outputFile);
		timer.stop();
		timer.printTimeInS();
	} else {
		cout << "grid_size num_threads STL_file output_file" << endl;
	}
}
