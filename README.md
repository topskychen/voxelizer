
## Overview


----------


This project voxelizes the meshes in STL file ***without*** the condition of *watertight*. It supports many formats only now, since we are using `assimp` library to load the file. Basically, the project can be summarized into two steps:

- Surface voxelization  
    For each piece of mesh (triangle) , we check the collided voxels in either way: 
    1. Get the minimal bounding box of each triangle, check each voxel in this box with the triangle;
    2. Start at any voxel collided with the triangle, and do bfs search to check neighboring voxels.   
    
    The first way is lightweight, but may become worse when the ratio of (triangle's volume/bounding box's volume) is small. While the second way has quite large constant overhead. For each thread in thread pool, it will pick a triangle to voxelize. The time complexity is O(m*c), where m is the triangle number and c is some factor such as the voxel number in the bounding box or constant overhead of bfs.
- Solid voxelization  
    When equipped with surface voxelization, the solid voxelization can be simple: flood fill. We try to flood fill the outer space of the meshes like carving the wood, since it is more simple and doesn't requires *watertight* property. However, the basic flood fill with bfs is too heavy and time-consuming, optimizations are proposed here (see below). The time complexity is O(n), where n is the voxel number in the bounding box of whole mesh.

## Optimizations
----------

- Use thread pool. 
- Use bit compression to record and order the voxel. First, I store voxel (x, y, z) to index=x\*size_y\*size_z + y\*size_z + z, where size_{x,y,z} are the voxel grid size, and I call the compressed format as *index*. For instance, with size=4, index=1 indicates voxel (0,0,1), index=4 indicates voxel (0,1,0). With indexes, all voxels can be represented with a binary string. For instance, '010010' means the voxels with indexes of 1 and 4 are collided with the mesh, while others are not. The binary string is further compressed with 64-bit unsigned int array.
- Use atomic<unsigned int> on the int array (the conceptional binary string) to guarantee the correctness when multi threads write the results, i.e., set the '0' or '1' to the int array. This design gives better parallel performance.
- Estimate the bounding box size of a triangle to guide which method is more suitable for surface voxelization.
- Randomly permutate the triangles' order to reduce the possibility of lock, thus get better parallel performance.
- Use filter-and-refine strategy to optimize the plain flood fill algorithm as follows: 
  - Filter  
  We fill the voxels along each axis, and stop when meet the mesh. For instance, fix (x,y), enumerate z in [0, maxz], if a voxel (x,y,z) is occupied, stop. The intuition is to mark all voxels which are visible along that axis. This idea is similar to flood fill but is much more efficient benefit from the lightweight operations. 
  - Refine  
  And it's worthy noting, after filtering, there are some *holes*, because they are not visible along any axis. But these are very few holes, so we can use flood fill again on them efficiently. 
  
  Although the time complexity is still O(n), it runs much faster than basic flood fill search.
- Compress the output file, for your convenience, I choose the simple version of output for default. But optimized version is prepared. Recall that the voxels are stored in binary string, such as '1110000'. To reduce the output file, please be noted only voxels in the minimal bounding box for mesh are output. It is further compressed as $value$ and $count$. For '1110000', it is compressed to (1)(2)(0)(3), (1)(2) means 3 consecutive 1, and (0)(3) means 4 consecutive 0. I use two bytes to record $value$ and $count$. Specifically, $value$ can be 0 or 1, and $count$ can be [0,255] which corresponds to [1,256]. To retrieve the coordination of a voxel in original space, let $(x,y,z)$ denote the voxel coordinate extracted from the output binary string, and the coordinate in voxelized space is $(x+x_{vox\_lb},y+y_{vox\_lb},z+z_{vox\_lb})$.
- [TODO] Combine the coarse and fine strategy, i.e., do coarse grid size first to filter more unnecessary voxels.
- [TODO] Use gpu
- [TODO] Try to fill the inside voxels, such that the time complexity is proportional to size of the result voxels.

## Installation


----------


This project requires libraries (dependencies) as follows:
- *absl* (https://github.com/abseil/abseil-cpp), which is installed as git submodule
- *boost*
- *libccd*
- *libfcl*    
  for collision checking (https://github.com/flexible-collision-library/fcl, version: tags/0.3.3)
- *assimp*  
    for loading STL file (https://github.com/assimp/assimp)
- *cmake & make*
    make sure `pkg-config` is installed.


CMakeLists.txt is used to generate makefiles. To build this project, in command line, run

``` cmake
mkdir build
cd build
cmake ..
```

Next, in linux, use `make` in 'build' directory to compile the code. 

## How to Use


----------

`voxelizer --help` will give all the parameters.

```Allowed options:
    --clipping_size (clipping size (x,y,z) to clip the voxelized result. The
      clipping size is grid size based. If only one integter is specified,
      clipping size is initialized as (x,x,x). If not set, the result is not
      clipped.); default: ;
    --format (output format, can be binvox or rawvox); default: "binvox";
    --grid_size (grid size, the granularity of voxelizer. if only one integer is
      set, assuming the X,Y,Z are the same.); default: ;
    --input (input file to be voxelized, file type will be inferred from file
      suffix); default: "";
    --mesh_index (mesh index to be voxelized); default: 0;
    --mode (voxelizer mode, surface or solid); default: "solid";
    --num_thread (number of thread to run voxelizer); default: 4;
    --output (output file to store voxelized result); default: "";
    --verbose (print debug info); default: false;
    --voxel_size (voxel size, which determines the size of each voxel);
      default: ;
    --with_meta (write voxel meta info to .meta file if set to true);
      default: false;
```

<!--- Output (voxel file format, it is wrote in `binvox` or `rawvox` mode, the 'read_rawvox.cpp' in test folder provides a sample code to load the .rawvox file.) `rawvox` is the following format:-->
<!--  - header-->
<!--    - size_x, size_y, size_z   -->
<!--    three integer denotes the size of grid system, e.g., 256, 256, 256-->
<!--    - lowerbound_x lowerbound_y lowerbound_z  -->
<!--    three doubles denote the lower bound of the original system, e.g., -0.304904 -0.304904 -0.304904-->
<!--    - voxel_size   -->
<!--    one double denotes the unit size of a voxel in original system, e.g., 0.00391916-->
<!--  - data-->
<!--    - x y z  -->
<!--    three integers denote the voxel coordinate in grid system, e.g, 30 66 194-->
<!--        - ...   -->
<!--When we have the voxel (x,y,z), we can get the box in original space as follows: (lowerbound_x + x\*voxel_size, lowerbound_y + y\*voxel_size, lowerbound_z + z\*voxel_size), (lowerbound_x + (x+1)\*voxel_size, lowerbound_y + (y+1)\*voxel_size, lowerbound_z + (z+1)\*voxel_size).-->

When you are in 'build' directory, one running example is: 

``` ./bin/voxelizer --input=../data/ironman.obj --output=../playground/ironman.binvox --grid_size=256 --verbose=true```
It outputs a `binvox` file with grid size `256 x 256 x 256`.

If you specify `--voxel_size=0.1`, instead, it will output a file with each voxel size is `0.1 x 0.1 x 0.1`. Note that the dimensions can be different.

To view the `ironman.binbox` file, you may use the viewvox here: https://www.patrickmin.com/viewvox/ or https://raahii.github.io/simple_voxel_viewer/index.html.


<!--For your reference, the pseudo output code for output is:-->

<!--```C++-->
<!--ofstream* output = new ofstream(p_file.c_str(), ios::out | ios::binary);-->
<!--*output << size_x << " " << size_y << " " << size_z << endl;-->
<!--*output << lowerbound_x << " " << lowerbound_y << " " << lowerbound_z << endl;-->
<!--*output << voxel_size << endl;-->
<!--for (x,y,z) in voxels:-->
<!--  *output << x << " " << y << " " << z << endl;-->
<!--```-->
    


<!--  - header
    - $x_{grid\size_}y_{grid\size_}z_{grid\size_}$
    three integer denote the grid sizes, e.g., 256 256 256
    - $x_{lb}y_{lb}z_{lb}$  
    three doubles denote the lower bounds of the original space, e.g., -0.304904 -0.304904 -0.304904
    - $x_{vox\unit_}y_{vox\unit_}z_{vox\unit_}$  
    three doubles denote the a voxel's size in original space, e.g., 0.00783833 0.00783833 0.00783833
    - $x_{vox\_lb}$$y_{vox\_lb}$$z_{vox\_lb}$
    three integers denote the lower bound of minimal bounding box in voxelized space, e.g., 30 0 8
        - $x_{vox\size_}$$y_{vox\size_}$$z_{vox\size_}$
    three integers denote the minimal bounding box's size in voxelized space, e.g., 
  - data  
    - $value_{01}count_{[0,255]}$...  
    Recall that the voxels are stored in binary string, such as '1110000'. To reduce the output file, please be noted only voxels in the minimal bounding box for mesh are output. It is further compressed as $value$ and $count$. For '1110000', it is compressed to (1)(2)(0)(3), (1)(2) means 3 consecutive 1, and (0)(3) means 4 consecutive 0. I use two bytes to record $value$ and $count$. Specifically, $value$ can be 0 or 1, and $count$ can be [0,255] which corresponds to [1,256]. To retrieve the coordination of a voxel in original space, let $(x,y,z)$ denote the voxel coordinate extracted from the output binary string, and the coordinate in voxelized space is $(x+x_{vox\_lb},y+y_{vox\_lb},z+z_{vox\_lb})$.
-->



## Directories


----------


This project has folders and files as follows:

 - *src*    
    the source files
 - *test*    
    the test files
 - *data*    
    testing data (.stl files)
 - *CMakeLists.txt*    
    for cmake
    
## Testing


----------

For simple testing, in `build` directory, run  ```./test/collision_checker```.
It is a benchmark of voxel collision checking and mesh collision checking (using the libfcl), with three testing files (kawada-hironx.stl, racecar.stl, and bike.stl). The grid size varies from 128 to 512, and the random cube (with random translate and random rotation) has size of 0.005^3 to 0.08^3 of the total scene size. The accuracy is defined based on the ground truth of mesh collision checking. As you can see, when the grid size becomes larger, the accuracy will increase. The collision checking performance is not specially optimized yet.

#### Testing Plaform
MacOS 10.9.3    
ubuntu 14.04