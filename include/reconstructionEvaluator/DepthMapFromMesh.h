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

class DepthMapFromMesh {
public:
  DepthMapFromMesh(Polyhedron *mesh);
  virtual ~DepthMapFromMesh();

  void setMeshGt(Polyhedron*& mesh) {
    mesh_ = mesh;
  }

  void computeMap(const CameraType &cam);

private:

  void computeRayFromCurCam(const float & x, const float &y, glm::vec3 &ray);
  void intersectRayMesh(const glm::vec3 &ray, glm::vec3 &intersection);

  Polyhedron *mesh_;
  CameraType curCam;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHMAPFROMMESH_H_ */
