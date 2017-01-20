/*
 * KittiOdoPoseGtParser.h
 *
 *  Created on: Dec 22, 2016
 *      Author: andrea
 */

#ifndef SRC_CAMPARSER_KITTIODOPOSEGTPARSER_H_
#define SRC_CAMPARSER_KITTIODOPOSEGTPARSER_H_

#include <ParserInterface.hpp>
namespace reconstructorEvaluator {

class KittiOdoPoseGtParser : ParserInterface {
public:
  KittiOdoPoseGtParser();
  virtual ~KittiOdoPoseGtParser();

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }
  void parse(const boost::filesystem::path &path);
  void setPathIntrinsicCalib(const boost::filesystem::path &path, int i);
private:
  glm::mat4 intrinsics_;
  int w,h;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_CAMPARSER_KITTIODOPOSEGTPARSER_H_ */
