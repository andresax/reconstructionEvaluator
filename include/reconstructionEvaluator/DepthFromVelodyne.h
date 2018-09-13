/*
 * DepthFromVelodyne.h
 *
 *  Created on: Dec 15, 2016
 *      Author: andrea
 */

#ifndef SRC_DEPTHFROMVELODYNE_H_
#define SRC_DEPTHFROMVELODYNE_H_
#include <string>
#include <vector>
#include <glm.hpp>
#include <CImg/CImg.h>

namespace reconstructorEvaluator {

class DepthFromVelodyne {
public:
  DepthFromVelodyne(const std::string &path, const int & imageHeight, const int & imageWidth );
  virtual ~DepthFromVelodyne() = default;

  void createDepthFromIdx(int idx);

  const cimg_library::CImg<float>& getDepth() const {
    return depth;
  }

  void setCalibCam(float cx, float cy, float fx, float fy);

private:
  std::string pathBase_;
  std::vector<glm::mat4> P;
  std::vector<glm::mat4> R_rect;
  glm::mat4 E_velo_to_cam;
  glm::mat4 tr;
  int imageHeight_;
  int imageWidth_;
  cimg_library::CImg<float> depth;
  bool odoSeq_;
  bool rawSeq_;
  float cx_, cy_, fx_, fy_;
  bool cameraSet;

  void loadCalib();
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHFROMVELODYNE_H_ */
