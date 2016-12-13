/*
 * CameraParser.h
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_CAMERAPARSER_H_
#define INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_CAMERAPARSER_H_

#include <boost/filesystem.hpp>
#include <types.hpp>
namespace reconstructorEvaluator {

class CameraParser {
public:
  CameraParser();
  virtual ~CameraParser();
  bool parseCameras(const boost::filesystem::path &path);

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }
private:

  std::vector<CameraType> cameras_;

};

} /* namespace reconstructorEvaluator */

#endif /* INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_CAMERAPARSER_H_ */
