/*
 * GtComparator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <GtComparator.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <Polyhedron_AssimpToCgal.h>
#include <assimp/Importer.hpp>  // OO version Header!
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace reconstructorEvaluator {

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
GtComparator::GtComparator(const std::string& path) {
  configuration_.setConfiguration(path);
  configuration_.parse();
}

GtComparator::~GtComparator() {

}

void GtComparator::run() {

  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(configuration_.getMeshPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  //    const aiScene* scene = importer.ReadFile(pFile, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices );

  aiMesh* mesh = scene->mMeshes[0];
  Polyhedron p;

  Builder_dae<HalfedgeDS> poly_builder(mesh);
  p.delegate(poly_builder);

  std::ofstream fileTest("testMesh.off");
  fileTest << p;

}

} /* namespace reconstructorEvaluator */
