/*
 * voxelizerMain.cpp
 *
 *  Created on: 1 Jul, 2014
 *      Author: chenqian
 */
#include "voxelizer.h"

using voxelizer::Timer;
using voxelizer::Voxelizer;

int main(int args, char* argv[]) {
  Timer timer;
  //	string fileName = "../data/kawada-hironx.stl";
  //	string fileName2 = "../data/test.binvox";
  if (args == 5) {
    int grid_size = atoi(argv[1]);
    int num_thread = atoi(argv[2]);
    string input_file = argv[3];
    string output_file = argv[4];
    timer.Restart();
    Voxelizer voxelizer(grid_size, input_file, true);
    timer.Stop();
    cout << "voxelizer initialization ";
    timer.PrintTimeInS();
    cout << "-------------------------------------------" << endl;
    timer.Restart();
    voxelizer.VoxelizeSurface(num_thread);
    timer.Stop();
    cout << "surface voxelization ";
    timer.PrintTimeInS();
    cout << "-------------------------------------------" << endl;
    timer.Restart();
    voxelizer.VoxelizeSolid(num_thread);
    timer.Stop();
    cout << "solid voxelization ";
    timer.PrintTimeInS();
    cout << "-------------------------------------------" << endl;
    timer.Restart();
    voxelizer.WriteSimple(output_file);
    //		voxelizer.write(output_file);
    //		voxelizer.writeForView(output_file);
    timer.Stop();
    cout << "writing file ";
    timer.PrintTimeInS();
    cout << "-------------------------------------------" << endl;
  } else {
    cout << "grid_size num_threads STL_file output_file" << endl;
  }
}
