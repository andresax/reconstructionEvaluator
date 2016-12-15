/*
 * DepthFromVelodyne.cpp
 *
 *  Created on: Dec 15, 2016
 *      Author: andrea
 */

#include "DepthFromVelodyne.h"

namespace reconstructorEvaluator {

DepthFromVelodyne::DepthFromVelodyne() {
  // TODO Auto-generated constructor stub

}

DepthFromVelodyne::~DepthFromVelodyne() {
  // TODO Auto-generated destructor stub
}

} /* namespace reconstructorEvaluator */

void reconstructorEvaluator::DepthFromVelodyne::createDepthFromIdx(int idx) {
//
//  _progress = iteration_index;
//    _current_rig = rig;
//    if(hasFinished()){
//      return true;
//    }
//    if(!rig){
//      _current_rig = &_scene->newCameraRig();
//    }
//    float dummy;
//    Cloud &new_cloud = _current_rig->getLidar().cloud;
//    Cloud::pointType const point;
//    std::fstream input(_filenames[_progress], std::ios::in | std::ios::binary);
//    while(input.good() && !input.eof()){
//      input.read((char *) &point.x, 3*sizeof(float));
//      input.read((char *) &dummy, sizeof(float));
//      if(    (std::abs(point.x) < _limit_x)
//        && (std::abs(point.y) < _limit_y)
//        && (std::abs(point.z) < _limit_z))
//      { new_cloud += point;  }
//    }
//    input.close();
//    if(new_cloud.size() == 0){
//      _scene->removeLastRig();
//      return false;
//    }
//    _current_rig->getLidar().timestamp = _timestamps[_progress];
//    _current_rig->getLidar().pose = _calibration;
//    return true;
}
