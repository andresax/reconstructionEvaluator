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
  void reset();

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }

  void setPathBaseGt(const boost::filesystem::path& pathBaseGt) {
    pathBaseGt_ = pathBaseGt;
  }

private:

  void convertToMvp2(CameraType &cam, glm::mat4 &mvpOut);
  void computeProjectionMatrix2(glm::mat3 &intrinsics, int h, int w, glm::mat4 &projectionMatrixOut);
  void computeModelViewMatrix2(glm::mat3 &rotation, glm::vec3 &translation, glm::mat4 &modelViewMatrixOut);
  std::vector<CameraType> cameras_;
  boost::filesystem::path pathBaseGt_;

};

} /* namespace reconstructorEvaluator */

#endif /* INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_CAMERAPARSER_H_ */
