/*
 * types.hpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef INCLUDE_RECONSTRUCTIONEVALUATOR_TYPES_HPP_
#define INCLUDE_RECONSTRUCTIONEVALUATOR_TYPES_HPP_

#include <glm.hpp>

struct CameraType {
  glm::mat3 intrinsics;
  glm::mat3 rotation;
  glm::vec3 translation;
  glm::mat4 cameraMatrix;
  glm::vec3 center;
  glm::vec3 distortion_coeff;

  int imageWidth;
  int imageHeight;
};



#endif /* INCLUDE_RECONSTRUCTIONEVALUATOR_TYPES_HPP_ */
