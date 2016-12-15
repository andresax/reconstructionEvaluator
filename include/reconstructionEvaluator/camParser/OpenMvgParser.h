/*
 * OpenMvgParser.h
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef CAM_PARSERS_OPENMVGPARSER_H_
#define CAM_PARSERS_OPENMVGPARSER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <types.hpp>
#include <rapidjson/document.h>



namespace reconstructorEvaluator {
class OpenMvgParser {
public:
  OpenMvgParser(std::string path);
  OpenMvgParser();
  virtual ~OpenMvgParser();

  void parse();

  void setFileName(const std::string& fileName) {
    fileName_ = fileName;
    fileStream_.open(fileName_.c_str());
  }

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }

private:
  void parseViews(const std::map<int,glm::mat3> & intrinsics, const std::map<int, glm::vec3> &distortion, const std::map<int,CameraType> & extrinsics);
  void parseIntrinsics(std::map<int,glm::mat3> & intrinsics, std::map<int, glm::vec3> &distortion);
  void parseExtrinsics(std::map<int,CameraType> & extrinsics);

  rapidjson::Document document_;
  std::string fileName_;
  std::ifstream fileStream_;
  std::vector<CameraType> cameras_;

};
}
#endif /* CAM_PARSERS_OPENMVGPARSER_H_ */
