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
#include <ParserInterface.hpp>



namespace reconstructorEvaluator {
class OpenMvgParser : ParserInterface{
public:
  OpenMvgParser();
  virtual ~OpenMvgParser();

  void setFileName(const std::string& fileName) {
    fileName_ = fileName;
    fileStream_.open(fileName_.c_str());
  }

  void parse(const boost::filesystem::path &path);

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

};
}
#endif /* CAM_PARSERS_OPENMVGPARSER_H_ */
