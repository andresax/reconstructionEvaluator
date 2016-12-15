/*
 * DepthFromVelodyne.h
 *
 *  Created on: Dec 15, 2016
 *      Author: andrea
 */

#ifndef SRC_DEPTHFROMVELODYNE_H_
#define SRC_DEPTHFROMVELODYNE_H_

namespace reconstructorEvaluator {

class DepthFromVelodyne {
public:
  DepthFromVelodyne();
  virtual ~DepthFromVelodyne();

  void createDepthFromIdx(int idx);
private:
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHFROMVELODYNE_H_ */
