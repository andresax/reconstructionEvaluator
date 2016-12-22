/*
 * EpflParser.h
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef SRC_CAMPARSER_EPFLPARSER_H_
#define SRC_CAMPARSER_EPFLPARSER_H_

#include <boost/filesystem.hpp>
#include <ParserInterface.hpp>
#include <types.hpp>
namespace reconstructorEvaluator {

class EpflParser : ParserInterface {
public:
  EpflParser();
  virtual ~EpflParser();

  void parse(const boost::filesystem::path &path);

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }

private:
  void orderPaths(const boost::filesystem::path &path);
  std::vector<boost::filesystem::path> cameraPaths_;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_CAMPARSER_EPFLPARSER_H_ */
