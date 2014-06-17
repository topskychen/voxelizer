## Installation

This project requires libraries as follow:
 - boost 
 - libfcl for collision checking (https://github.com/flexible-collision-library/fcl)
 - assimp for loading STL file (https://github.com/assimp/assimp)


CMakeLists.txt is used to generate makefiles. To build this project, in command line, run
``` cmake
mkdir build
cd build
cmake ..
```
Next, in linux, use make to compile the code. 

## Directories
This project has folders and files as follow:
 - src, the source files
 - data, testing data
 - CMakeLists.txt, for cmake
