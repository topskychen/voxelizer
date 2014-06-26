
## Overview
===========
This project voxelizes the meshes in STL file ***without*** the condition of *watertight*. Basically, the project can be summarized into two steps:

- Surface voxelization
    For each piece of mesh (triangle) , we check the collided voxels as in either way: 
    1. get the minimal bounding box of each triangle, check each voxel in this box with the triangle;
    2. start any voxel on the triangle, and do bfs search to check neighboring voxels. 
    
    The first way is lightweight, but may become worse when the triangle's box is extremely large. While the second way has quite large constant overhead. For each triangle, we apply a thread. The time complexity is O(m*c), where m is the triangle number and c is the voxel number in the bounding box.
- Solid voxelization
    Equipped with surface voxelization, the basic idea for solid voxelization is simple: flood fill. We try to foold fill the outer space of the meshes, since it is more simple and doesn't requires *watertight* property. However, the basic flood fill with bfs is too heavy, optimizations are proposed here. The time complexity is O(n), where n is the voxel number in the bounding box of whole mesh.

#####Optimizations

- Use thread pool. 
- Bit compression to record the voxel information. (x, y, z) -> x\*size\*size + y\*size + size, and it is further compressed with 32-bit unsigned int. Bit 1 means this location is occupied, and 0 means not.
- Use atomic<> for each voxel to guarantee the correctness when multi threads write the results, and it gives better parallel performance.
- Estimate the bounding box size of a triangle to guide the right method to check collision.
- The plain flood fill algorithm is time-consuming. I optimize it as follows. We fill the voxels along each axis. For instance, fix (x, y), enumerate z in [0, maxz], if some a voxel at (x, y, z) is occupied by surface voxel, break the loop. Like lighting along z axis, and stop when meeting the surface. This idea is similar to flood fill but is much more efficient. And it's worthy noting, after this algorithm, there are some *holes*. But these holes are only a few, so we can use flood fill again on them.
- [TODO] gpu
- [TODO] try to fill the inside voxels, such that the time complexity is proportional to size of the result voxels.

## Installation
===============

This project requires libraries (dependencies) as follow:

- *boost* 
- *libfcl* 
    for collision checking (https://github.com/flexible-collision-library/fcl), no octree required
- *assimp* 
    for loading STL file (https://github.com/assimp/assimp)
- *cmake & make*


CMakeLists.txt is used to generate makefiles. To build this project, in command line, run

``` cmake
mkdir build
cd build
cmake ..
```

Next, in linux, use `make` to compile the code. 

## How to use
=============

- Input
	- grid size, e.g., 256
	- number of threads, e.g., 4
	- the STL file, e.g., kawada-hironx.stl
	- the output file, e.g., kawada-hironx.vox
- Output (voxel file format)	
	- header
		- dim_x dim_y dim_z (e.g., 256 256 256)
		- lowerbound_x lowerbound_y lowerbound_z (e.g., )
		- voxel_size (e.g., )
	- data
		- [byte_value][byte_count] 		
    	Please be noted there are no brackets and all voxels (x,y,z) are ordered with x\*dim_y\*dim_z+y*dim_z+z. e.g., [0][255] means all consecutive 255 voxels are not filled. The output is in binary.
		- ...

C++ code for output:

```C++
ofstream* outp = new ofstream(pFile.c_str(), ios::out | ios::binary);
*out << dim_x << " " << dim_y << " " << dim_z << endl;
*out << lowerbound_x << " " << lowerbound_y << " " << lowerbound_z << endl;
*out << voxel_size << endl;
while (index < totalSize) {
	value = getValue(index);
	count = 0;
	while ((index < totalSize) && (count < 255) && (value == getValue(index))) {
		index++;
		count++;
	}
	*out << value << count;
	bytes_written += 2;
}
```

An example is: `./Voxelizer 256 4 ../data/kawada-hironx.stl ../data/kawada-hironx.vox`.
## Directories
==============
This project has folders and files as follow:

 - *src*
    the source files
 - *data* 
    testing data (.stl files)
 - *CMakeLists.txt* 
    for cmake