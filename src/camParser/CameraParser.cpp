/*
 * CameraParser.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <CameraParser.h>
#include <EpflParser.h>
#include <OpenMvgParser.h>
#include <KittiOdoPoseGtParser.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace reconstructorEvaluator {

CameraParser::CameraParser() {
}

CameraParser::~CameraParser() {
}

bool CameraParser::parseCameras(const boost::filesystem::path &path) {

  std::cout<<"parsing..."<<path.string()<<std::endl;
  if (boost::filesystem::exists(path)) {
    if (boost::filesystem::is_directory(path)) {
      boost::filesystem::path::iterator it = path.begin();
      if (path.parent_path().filename().string() == "CamerasKE") {
        EpflParser epflParser;
        epflParser.parse(path);
        cameras_ = epflParser.getCameras();

      }else {
        std::cout << "CameraParser::parseCameras ERROR: path is a directory but it does not link to a CamerasKE folder";
        std::cout << ";  " << path.parent_path().filename().string() << " instead" << std::endl;
        return false;

      }
    }else if (boost::filesystem::is_regular_file(path)) {
      if(boost::filesystem::extension(path) == ".json"){
        OpenMvgParser ompars;
        ompars.parse(path);
        cameras_ = ompars.getCameras();

      }else if (boost::filesystem::extension(path) == ".txt") {
        KittiOdoPoseGtParser kittiParser;
        kittiParser.setPathIntrinsicCalib(pathBaseGt_, 0);
        kittiParser.parse(path);
        cameras_ = kittiParser.getCameras();

      } else {
        std::cout << "CameraParser::parseCameras ERROR: path is not a json or txt file" << std::endl;
        return false;
      }
    } else {
      std::cout << "CameraParser::parseCameras ERROR: path is not a directory or a regular file" << std::endl;
      return false;
    }
  } else {
    std::cout << "CameraParser::parseCameras ERROR: path does not exist" << std::endl;
    return false;
  }
  return true;

}
} /* namespace reconstructorEvaluator */

void reconstructorEvaluator::CameraParser::reset() {
  cameras_.clear();
}
