/*
 * DepthMapFromMesh.h
 *
 *  Created on: Dec 14, 2016
 *      Author: andrea
 */

#ifndef SRC_DEPTHMAPFROMMESH_H_
#define SRC_DEPTHMAPFROMMESH_H_

#include <types.hpp>
#include <CImg/CImg.h>
namespace reconstructorEvaluator {

typedef Kernel::Ray_3 Ray;
class DepthMapFromMesh {
public:
  DepthMapFromMesh(Polyhedron *mesh);
  virtual ~DepthMapFromMesh();

  void setMeshGt(Polyhedron*& mesh) {
    mesh_ = mesh;
  }

  void computeMap(const CameraType &cam);

  const cimg_library::CImg<float>& getDepth() const {
    return depth;
  }

private:

  void computeRayFromCurCam(const float & x, const float &y, glm::vec3 &ray);

  void printRays(const std::vector<Ray> &rays);

  Polyhedron *mesh_;
  CameraType curCam;
  cimg_library::CImg<float> depth;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHMAPFROMMESH_H_ */
