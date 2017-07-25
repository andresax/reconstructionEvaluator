/*
 * GtComparator.h
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef SRC_GTCOMPARATOR_H_
#define SRC_GTCOMPARATOR_H_

#include <string>
#include <vector>
#include <types.hpp>
#include <Configuration.h>
#include <CImg/CImg.h>
#undef Success
#include <Eigen/Core>

namespace reconstructorEvaluator {

class GtComparator {
public:
  GtComparator(const std::string &path);
  virtual ~GtComparator();
  void run();
  void run2();
private:
  void importGT();
  void importMesh();
  void registerCameras();
  void compareDepthMaps(const std::vector<cimg_library::CImg<float>> &depthGT, const std::vector<cimg_library::CImg<float>> &depth);
  void compareDepthMaps();
  void accumulateDepthMaps(const cimg_library::CImg<float> &depthGT, const cimg_library::CImg<float> &depth);
  void printComparison();
  void loadDepthMapGT(const std::string &pathGT_, cimg_library::CImg<float> &depth,int w,int h);
  Configuration configuration_;

  Polyhedron meshGt_;
  Polyhedron meshToBeCompared_;
  ComparisonResults res;
  Eigen::Matrix4f rotoTr_;
  Eigen::Matrix3f Rsub;
  Eigen::Vector3f T;
  float scale_;
  int countImages_;

};

} /* namespace reconstructorEvaluator */

#endif /* SRC_GTCOMPARATOR_H_ */
