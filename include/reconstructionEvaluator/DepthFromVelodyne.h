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

namespace reconstructorEvaluator {

class DepthFromVelodyne {
public:
  DepthFromVelodyne(const std::string &path, const int & imageHeight, const int & imageWidth );
  virtual ~DepthFromVelodyne();

  void createDepthFromIdx(int idx);

private:
  std::string pathBase_;
  std::vector<glm::mat4> P;
  glm::mat4 tr;
  int imageHeight_;
  int imageWidth_;

  void loadCalib();
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHFROMVELODYNE_H_ */
