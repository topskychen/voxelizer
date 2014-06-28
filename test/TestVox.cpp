#include <iostream>
#include <fstream>
#include <string>
using namespace std;


int main(int argc, char* argv[]) {
	ifstream* input = new ifstream(argv[1], ios::in | ios::binary);
	int grid_size;
	double lx, ly, lz, voxel_size;
	*input >> grid_size;
	*input >> lx >> ly >> lz;
	*input >> voxel_size;
	int x, y, z, count = 0;
	while (*input >> x >> y >> z) {
		// do something
		if (count == 0) cout << "first line : " << x << ", " << y << ", " << z << endl; // first line
		++count;
	}
	cout << "last line : " << x << ", " << y << ", " << z << endl; //last line
	input->close();
	cout << "grid size : " << grid_size << endl;
	cout << "lower bound : " << lx << " " << ly << " " << lz << endl;
	cout << "voxel size : " << voxel_size << endl;
	cout << "voxel count : " << count << endl;
}

