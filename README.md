
## Overview

This project voxelizes the meshes in STL file ***without*** the condition of *watertight*. It supports *.stl files only. Basically, the project can be summarized into two steps:

- Surface voxelization  
    For each piece of mesh (triangle) , we check the collided voxels in either way: 
    1. get the minimal bounding box of each triangle, check each voxel in this box with the triangle;
    2. start at any voxel collided with the triangle, and do bfs search to check neighboring voxels.   
    
    The first way is lightweight, but may become worse when the ratio of (triangle's volume/bounding box's volume) is small. While the second way has quite large constant overhead. For each thread in thread pool, it will pick a triangle to solve. The time complexity is O(m*c), where m is the triangle number and c is some facter such as the voxel number in the bounding box or constant overhead of bfs.
- Solid voxelization  
    When equipped with surface voxelization, the solid voxelization can be simple: flood fill. We try to foold fill the outer space of the meshes, since it is more simple and doesn't requires *watertight* property. However, the basic flood fill with bfs is too heavy and time-consuming, optimizations are proposed here (see below). The time complexity is O(n), where n is the voxel number in the bounding box of whole mesh.

#####Optimizations

- Use thread pool. 
- Use bit compression to record and order the voxel. First, I store voxel (x, y, z) to index=x\*size\*size + y\*size + size, where size is the voxel grid size, and I call the compressed format as *index*. For instance, with size=4, index=1 means voxel (0,0,1), index=4 means voxel (0, 1, 0). Thus, all voxels can be represented with a binary string. For instance, '010010' means the voxels with indexes of 1 and 4 are collided with the mesh, while others are not. The binary string is further compressed with 32-bit unsigned int.
- Use atomic<unsigned int> on the binary string (actually they are unsigned int arrays) to guarantee the correctness when multi threads write the results, i.e., set the '0' or '1' to the bianry string. It gives better parallel performance.
- Estimate the bounding box size of a triangle to guide the right method to check collision in surface mesh.
- Random permutate the triangles' order to reduce the possibility of lock, thus get better parallel performance.
- Use filter-and-refine strategy to optimize the plain flood fill algorithm as follows. 
	- Filter  
	We fill the voxels along each axis, and stop when meet the mesh. For instance, fix (x, y), enumerate z in [0, maxz], if a voxel (x, y, z) is occupied, stop. The intuition is to mark all voxels which are visible along that axis. This idea is similar to flood fill but is much more efficient. 
	- Refine  
	And it's worthy noting, after filtering, there are some *holes*, since they are not visible along any axis. But these holes are of very few numbers, so we can use flood fill again on them efficiently. 
	
	Although the time complexity is still O(n), it runs much faster than basic flood fill search.
- [TODO] Combine the coarse and fine strategy, i.e., do coarse grid size first to filter more unnecessary voxels.
- [TODO] Use gpu
- [TODO] Try to fill the inside voxels, such that the time complexity is proportional to size of the result voxels.

## Installation


This project requires libraries (dependencies) as follow:

- *boost* 
- *libfcl* 		
	for collision checking (https://github.com/flexible-collision-library/fcl), ***libccd*** is required
- *assimp*  
    for loading STL file (https://github.com/assimp/assimp)
- *cmake & make*


CMakeLists.txt is used to generate makefiles. To build this project, in command line, run

``` cmake
mkdir build
cd build
cmake ..
```

Next, in linux, use `make` in 'build' directory to compile the code. 

## How to use


- Input
	- grid size, e.g., 256
	- number of threads, e.g., 4
	- the STL file, e.g., kawada-hironx.stl
	- the output file, e.g., kawada-hironx.vox
- Output (voxel file format, it is wrote in ***bianry*** mode)
	- header
		- 'grid_size' (one integer, e.g., '256')
		- 'lowerbound_x''lowerbound_y''lowerbound_z' (three doubles, e.g., '-0.304904''-0.304904''-0.304904')
		- 'voxel_size' (one double, e.g., '0.00391916')
	- data
		- 'x''y''z'... (the voxel coordinate in grid system, e.g, '0''1''0')

For your reference, the pesudo output code is:
'''C++
ofstream* output = new ofstream(pFile.c_str(), ios::out | ios::binary);
*output << grid_size;
*output << lowerbound_x << lowerbound_y << lowerbound_z;
*output << voxel_size;
for (x,y,z) in voxels:
	*output << x << y << z;
'''

When you are in 'build' directory, an example is: `./bin/Voxelizer 256 4 ../data/kawada-hironx.stl ../data/kawada-hironx.vox`.

## Directories

This project has folders and files as follow:

 - *src*
    the source files
 - *data* 
    testing data (.stl files)
 - *CMakeLists.txt* 
    for cmake