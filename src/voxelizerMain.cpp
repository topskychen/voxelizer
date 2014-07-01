/*
 * voxelizerMain.cpp
 *
 *  Created on: 1 Jul, 2014
 *      Author: chenqian
 */
#include "voxelizer.h"

int main(int args, char* argv[]) {
	Timer timer;
//	string fileName = "../data/kawada-hironx.stl";
//	string fileName2 = "../data/test.binvox";
	if (args == 5) {
		int gridSize = atoi(argv[1]);
		int numThread = atoi(argv[2]);
		string inputFile = argv[3];
		string outputFile = argv[4];
		timer.Restart();
		Voxelizer voxelizer(gridSize, inputFile, true);
		timer.Stop();
		cout << "voxelizer initialization "; timer.PrintTimeInS();
		cout << "-------------------------------------------" << endl;
		timer.Restart();
		voxelizer.VoxelizeSurface(numThread);
		timer.Stop();
		cout << "surface voxelization "; timer.PrintTimeInS();
		cout << "-------------------------------------------" << endl;
		timer.Restart();
		voxelizer.VoxelizeSolid(numThread);
		timer.Stop();
		cout << "solid voxelization "; timer.PrintTimeInS();
		cout << "-------------------------------------------" << endl;
		timer.Restart();
		voxelizer.WriteSimple(outputFile);
//		voxelizer.write(outputFile);
//		voxelizer.writeForView(outputFile);
		timer.Stop();
		cout << "writing file "; timer.PrintTimeInS();
		cout << "-------------------------------------------" << endl;
	} else {
		cout << "grid_size num_threads STL_file output_file" << endl;
	}
}

