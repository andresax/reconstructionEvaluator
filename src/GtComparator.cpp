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

  std::cout << "GtComparator:: writing mesh...";
  std::cout.flush();
  std::ofstream fileTest1("testMesh.off");
  fileTest1 << meshToBeCompared_;
  std::cout << "DONE." << std::endl;

//  std::cout<<"GtComparator:: writing mesh gt...";
//  std::cout.flush();
//  std::ofstream fileTest2("testGT.off");
//  fileTest2 << meshGt_;
//  std::cout<<"DONE."<<std::endl;

  DepthMapFromMesh dmfm(&meshToBeCompared_);
  dmfm.computeMap(configuration_.getCameras()[10]);
  DepthFromVelodyne frv(configuration_.getGtPath(), configuration_.getCameras()[0].imageHeight, configuration_.getCameras()[0].imageWidth);

  frv.createDepthFromIdx(10);

  compareDepthMaps(frv.getDepth(), dmfm.getDepth());
  printComparison();
}

void GtComparator::importGT() {

  Assimp::Importer importer;

  std::cout << "GtComparator::importGT importing...";
  std::cout.flush();
  const aiScene* scene = importer.ReadFile(configuration_.getGtPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  std::cout << "DONE." << std::endl;

  aiMesh* mesh = scene->mMeshes[0];

  std::cout << "GtComparator::poly_builderGT importing...";
  std::cout.flush();
  MeshBuilder<HalfedgeDS> poly_builder(mesh);
  meshGt_.delegate(poly_builder);
  std::cout << "DONE." << std::endl;

}

void GtComparator::importMesh() {

  Assimp::Importer importer;

  std::cout << "GtComparator::importMesh importing...";
  std::cout.flush();
  const aiScene* scene = importer.ReadFile(configuration_.getMeshPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  std::cout << "DONE." << std::endl;

  aiMesh* mesh = scene->mMeshes[0];

  std::cout << "GtComparator::poly_builder importing...";
  std::cout.flush();
  MeshBuilder<HalfedgeDS> poly_builder(mesh);
  meshToBeCompared_.delegate(poly_builder);
  std::cout << "DONE." << std::endl;
}

void GtComparator::compareDepthMaps(const cimg_library::CImg<float>& depthGT, const cimg_library::CImg<float>& depth) {

  if (depthGT._width != depth._width || depthGT._height != depth._height) {
    std::cout << " compareDepthMaps error the two depth maps have different dimensions" << std::endl;
    return;
  }

  for (int x = 0; x < depthGT._width; ++x) {
    for (int y = 0; y < depthGT._height; ++y) {
      if (depth(x, y) > 0.0 && depthGT(x, y) > 0.0)
        res.errs_.push_back(depth(x, y) - depthGT(x, y));
    }
  }

  float sum = std::accumulate(res.errs_.begin(), res.errs_.end(), 0.0);

  std::vector<float> sqrVec;
  std::transform(res.errs_.begin(), res.errs_.end(), std::back_inserter(sqrVec), [](int n) {return std::pow(n,2);});
  float sumSqr = std::accumulate(res.errs_.begin(), res.errs_.end(), 0.0);
  std::vector<float> absVec;
  std::transform(res.errs_.begin(), res.errs_.end(), std::back_inserter(absVec), [](int n) {return std::fabs(n);});
  float sumAbs = std::accumulate(res.errs_.begin(), res.errs_.end(), 0.0);

  res.mean = sum / res.errs_.size();
  res.rmse = std::sqrt(sumSqr) / res.errs_.size();
  res.mae = sumAbs / res.errs_.size();

  std::vector<double> diff(res.errs_.size());
  std::transform(res.errs_.begin(), res.errs_.end(), diff.begin(), std::bind2nd(std::minus<double>(), res.mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  res.stddev = std::sqrt(sq_sum / res.errs_.size());
}

void GtComparator::printComparison() {

  std::cout << "Error statistics: " << std::endl;
  std::cout << "mean: " << res.mean << " stddev: " << res.stddev << std::endl;
  std::cout << "RMSE: " << res.rmse << " mae: " << res.mae << std::endl;

  float maxEl = *std::max_element(res.errs_.begin(), res.errs_.end());
  float minEl = *std::min_element(res.errs_.begin(), res.errs_.end());
  const float binSize = 10.0;
  int numBin = (int) ceil((maxEl - minEl) / binSize); // requires <cmath>
  std::vector<int> histError(numBin, 0);

  for (auto e : res.errs_) {
    int bucket = (int) floor(fabs(e) / binSize);

    histError[bucket < numBin ? bucket : numBin - 1] += 1;
  }

  int totNum = std::accumulate(histError.begin(), histError.end(), 0);

  std::cout << "Errors histogram (%): " << std::endl;
  for (auto c : histError) {
    std::cout << setfill(' ') << setw(10) << std::fixed << std::setprecision(1);
    std::cout << 100.0 * static_cast<float>(c) / totNum << " ";
  }
  std::cout << std::endl;
  std::cout << "Errors histogram: (num)" << std::endl;
  for (auto c : histError) {
    std::cout << setfill(' ') << setw(10) << std::fixed << std::setprecision(1);
    std::cout << c << " ";
  }
  std::cout << std::endl;

}

} /* namespace reconstructorEvaluator */
