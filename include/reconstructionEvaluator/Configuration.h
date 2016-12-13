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
  virtual ~Configuration();
  void parse();
private:
  std::ifstream file_;
  bool parseWhichCams(const std::string &path);
  std::vector<CameraType> cameras_;
  std::vector<int> camerasIdx_;


};

} /* namespace reconstructorEvaluator */

#endif /* SRC_CONFIGURATION_H_ */
