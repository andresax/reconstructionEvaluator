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

namespace reconstructorEvaluator {

typedef Polyhedron::HalfedgeDS HalfedgeDS;
GtComparator::GtComparator(const std::string& path) {
  configuration_.setConfiguration(path);
  configuration_.parse();
}

GtComparator::~GtComparator() {

}

void GtComparator::run() {
  importGT();
  importMesh();

  std::ofstream fileTest1("testMesh.off");
  fileTest1 << meshToBeCompared_;
  std::ofstream fileTest2("testGT.off");
  fileTest2 << meshGt_;

}

void GtComparator::importGT() {

  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(configuration_.getMeshPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  //    const aiScene* scene = importer.ReadFile(pFile, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices );

  aiMesh* mesh = scene->mMeshes[0];

  Builder_dae<HalfedgeDS> poly_builder(mesh);
  meshGt_.delegate(poly_builder);
}

void GtComparator::importMesh() {

  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(configuration_.getGtPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  //    const aiScene* scene = importer.ReadFile(pFile, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices );

  aiMesh* mesh = scene->mMeshes[0];

  Builder_dae<HalfedgeDS> poly_builder(mesh);
  meshToBeCompared_.delegate(poly_builder);
}

} /* namespace reconstructorEvaluator */
