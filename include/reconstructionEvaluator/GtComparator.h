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
private:
  void importGT();
  void importMesh();
  void registerCameras();
  void estimateScale();
  void compareDepthMaps(const cimg_library::CImg<float> &depthGT, const cimg_library::CImg<float> &depth);
  void printComparison();
  Configuration configuration_;

  Polyhedron meshGt_;
  Polyhedron meshToBeCompared_;
  ComparisonResults res;
  Eigen::Matrix4f rotoTr_;

};

} /* namespace reconstructorEvaluator */

#endif /* SRC_GTCOMPARATOR_H_ */
