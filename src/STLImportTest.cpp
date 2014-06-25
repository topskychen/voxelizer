//============================================================================
// Name        : mujin.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <boost/shared_ptr.hpp>
#include <fcl/collision.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/AABB.h>
#include <iostream>
using namespace std;

typedef boost::shared_ptr<aiScene> scene_p;
typedef boost::shared_ptr<aiMesh> mesh_p;
typedef boost::shared_ptr<int> int_p;


bool importMesh(const string& pFile, const aiScene*& scene) {

  Assimp::Importer importer;
  scene = importer.ReadFile(pFile, aiProcessPreset_TargetRealtime_Fast);

  if (!scene) {
    cerr << "Scene fails to be loaded!" << endl;
    cerr << importer.GetErrorString() << endl;
    return false;
  }
  return true;
}

int main(int args, char* argv[]) {
  const aiScene* scene;
  importMesh("/Users/chenqian/workspace/mujin/data/kawada-hironx.stl", scene);
  fcl::AABB a;
}
