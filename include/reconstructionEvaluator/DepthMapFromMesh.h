/*
 * DepthMapFromMesh.h
 *
 *  Created on: Dec 14, 2016
 *      Author: andrea
 */

#ifndef SRC_DEPTHMAPFROMMESH_H_
#define SRC_DEPTHMAPFROMMESH_H_

#include <types.hpp>
namespace reconstructorEvaluator {

class DepthMapFromMesh {
public:
  DepthMapFromMesh();
  virtual ~DepthMapFromMesh();

  void setMeshGt(const Polyhedron*& mesh) {
    mesh_ = mesh;
  }

private:

  Polyhedron *mesh_;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHMAPFROMMESH_H_ */
