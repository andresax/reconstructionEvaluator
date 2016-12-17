/*
 * types.hpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef INCLUDE_RECONSTRUCTIONEVALUATOR_TYPES_HPP_
#define INCLUDE_RECONSTRUCTIONEVALUATOR_TYPES_HPP_

#include <glm.hpp>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
//#include <CImg/CImg.h>

struct CameraType {
  glm::mat3 intrinsics;
  glm::mat4 extrinsics;
  glm::mat3 rotation;
  glm::vec3 translation;
  glm::mat4 cameraMatrix;
  glm::vec3 center;
  glm::vec3 distortion_coeff;

  int imageWidth;
  int imageHeight;
};

struct ComparisonResults{
  float mean;
  float stddev;
  float mae;
  float rmse;

  std::vector<float> errs_;
  //std::vector<cimg_library::CImg<float>> mapsErr;


};

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;


#endif /* INCLUDE_RECONSTRUCTIONEVALUATOR_TYPES_HPP_ */
