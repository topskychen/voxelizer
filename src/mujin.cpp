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


