/*
 * Configuration.h
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef SRC_CONFIGURATION_H_
#define SRC_CONFIGURATION_H_

#include <string>
#include <vector>
#include <fstream>
#include <types.hpp>

namespace reconstructorEvaluator {

class Configuration {
public:
  Configuration(const std::string &path);
  Configuration();
  virtual ~Configuration();
  void setConfiguration(const std::string &path);
  void parse();

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }

  const std::vector<int>& getCamerasIdx() const {
    return camerasIdx_;
  }

  const std::string& getMeshPath() const {
    return meshPath_;
  }

  const std::string& getGtPath() const {
    return gtPath_;
  }

private:
  std::ifstream file_;
  bool parseWhichCams(const std::string &path);

  std::string meshPath_;
  std::string gtPath_;
  std::vector<CameraType> cameras_;
  std::vector<int> camerasIdx_;

};

} /* namespace reconstructorEvaluator */

#endif /* SRC_CONFIGURATION_H_ */
