/*
 * GtComparator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <GtComparator.h>
#include <Polyhedron_AssimpToCgal.h>
#include <assimp/Importer.hpp>  // OO version Header!
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <DepthMapFromMesh.h>
#include <DepthFromVelodyne.h>


namespace reconstructorEvaluator {

typedef Polyhedron::HalfedgeDS HalfedgeDS;
GtComparator::GtComparator(const std::string& path) {
  configuration_.setConfiguration(path);
  configuration_.parse();
}

GtComparator::~GtComparator() {

}

void GtComparator::run() {
  //importGT();
  //importMesh();
  std::ifstream file(configuration_.getMeshPath());
  file >> meshToBeCompared_;

  std::cout<<"GtComparator:: writing mesh...";
  std::cout.flush();
  std::ofstream fileTest1("testMesh.off");
  fileTest1 << meshToBeCompared_;
  std::cout<<"DONE."<<std::endl;


//  std::cout<<"GtComparator:: writing mesh gt...";
//  std::cout.flush();
//  std::ofstream fileTest2("testGT.off");
//  fileTest2 << meshGt_;
//  std::cout<<"DONE."<<std::endl;

  DepthMapFromMesh dmfm(&meshToBeCompared_);
  dmfm.computeMap(configuration_.getCameras()[0]);
  DepthFromVelodyne frv(configuration_.getGtPath(), configuration_.getCameras()[0].imageHeight, configuration_.getCameras()[0].imageWidth);

  frv.createDepthFromIdx(0);
}

void GtComparator::importGT() {

  Assimp::Importer importer;

  std::cout<<"GtComparator::importGT importing...";
  std::cout.flush();
  const aiScene* scene = importer.ReadFile(configuration_.getGtPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  std::cout<<"DONE."<<std::endl;

  aiMesh* mesh = scene->mMeshes[0];

  std::cout<<"GtComparator::poly_builderGT importing...";
  std::cout.flush();
  MeshBuilder<HalfedgeDS> poly_builder(mesh);
  meshGt_.delegate(poly_builder);
  std::cout<<"DONE."<<std::endl;

}

void GtComparator::importMesh() {

  Assimp::Importer importer;

  std::cout<<"GtComparator::importMesh importing...";
  std::cout.flush();
  const aiScene* scene = importer.ReadFile(configuration_.getMeshPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  std::cout<<"DONE."<<std::endl;

  aiMesh* mesh = scene->mMeshes[0];

  std::cout<<"GtComparator::poly_builder importing...";
  std::cout.flush();
  MeshBuilder<HalfedgeDS> poly_builder(mesh);
  meshToBeCompared_.delegate(poly_builder);
  std::cout<<"DONE."<<std::endl;
}

} /* namespace reconstructorEvaluator */
