/*
 * Configuration.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <Configuration.h>
#include <utilities.hpp>
#include <CameraParser.h>

namespace reconstructorEvaluator {

Configuration::Configuration(const std::string &path) {
  file_.open(path.c_str());
  if (file_.is_open())
    std::cout << path << " opened" << std::endl;
  else
    std::cout << "Error reading " << path << std::endl;
}

Configuration::~Configuration() {
}
void Configuration::parse() {

  CameraParser cameraParser;
  std::string gt;
  std::string meshPath;
  std::string cameraPoses;
  std::string whichCams;
  std::string camerasGT;

  utilities::readLineAndStore(file_, gt);
  utilities::readLineAndStore(file_, meshPath);
  utilities::readLineAndStore(file_, cameraPoses);
  if (!cameraParser.parseCameras(cameraPoses)) {
    std::cout << "Configuration::parse ERROR parsing cameras" << std::endl;
  } else {
    cameras_ = cameraParser.getCameras();
  }
  utilities::readLineAndStore(file_, whichCams);
  if (!parseWhichCams(whichCams)) {
    std::cout << "Configuration::parse ERROR parsing whichCams" << std::endl;
  }

  for(auto i:camerasIdx_){
    std::cout << i<<std::endl;
  }
  utilities::readLineAndStore(file_, camerasGT);

}

bool Configuration::parseWhichCams(const std::string& whichCams) {
  if (whichCams == "") {
    for (int idx = 0; idx < cameras_.size(); ++idx) {
      camerasIdx_.push_back(idx);
    }
  } else {
    std::ifstream file(whichCams);
    if (file.is_open()) {
      std::cout << whichCams << " opened" << std::endl;
    } else {
      std::cout << "Error reading " << whichCams << std::endl;
      return false;
    }
    int idx;
    while (!file.eof()) {
      file >> idx;
      camerasIdx_.push_back(idx);
    }
  }
  return true;
}

} /* namespace reconstructorEvaluator */
