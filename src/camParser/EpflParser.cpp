/*
 * EpflParser.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <EpflParser.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <utilities.hpp>

namespace reconstructorEvaluator {

EpflParser::EpflParser() {

}

EpflParser::~EpflParser() {
}

void EpflParser::parse(const boost::filesystem::path& path) {
  orderPaths(path);
int i=0;
  for (auto camPath : cameraPaths_) {
    CameraType curCam;
    std::ifstream camFile(camPath.string());
    camFile >> curCam.intrinsics[0][0] >> curCam.intrinsics[0][1] >> curCam.intrinsics[0][2];
    camFile >> curCam.intrinsics[1][0] >> curCam.intrinsics[1][1] >> curCam.intrinsics[1][2];
    camFile >> curCam.intrinsics[2][0] >> curCam.intrinsics[2][1] >> curCam.intrinsics[2][2];
    camFile >> curCam.distortion_coeff[0] >> curCam.distortion_coeff[1] >> curCam.distortion_coeff[2];
    //Reading the transpose rotation  since the file stores the inverse of the Hartley Zisserman convention
    camFile >> curCam.rotation[0][0] >> curCam.rotation[1][0] >> curCam.rotation[2][0];
    camFile >> curCam.rotation[0][1] >> curCam.rotation[1][1] >> curCam.rotation[2][1];
    camFile >> curCam.rotation[0][2] >> curCam.rotation[1][2] >> curCam.rotation[2][2];
    camFile >> curCam.center[0] >> curCam.center[1] >> curCam.center[2];


    curCam.translation = -curCam.center * curCam.rotation;

    glm::mat4 tempCameraExtrinsic(0.0);
    tempCameraExtrinsic[0][0] = curCam.rotation[0][0];
    tempCameraExtrinsic[0][1] = curCam.rotation[0][1];
    tempCameraExtrinsic[0][2] = curCam.rotation[0][2];
    tempCameraExtrinsic[1][0] = curCam.rotation[1][0];
    tempCameraExtrinsic[1][1] = curCam.rotation[1][1];
    tempCameraExtrinsic[1][2] = curCam.rotation[1][2];
    tempCameraExtrinsic[2][0] = curCam.rotation[2][0];
    tempCameraExtrinsic[2][1] = curCam.rotation[2][1];
    tempCameraExtrinsic[2][2] = curCam.rotation[2][2];
    tempCameraExtrinsic[0][3] = curCam.translation[0];
    tempCameraExtrinsic[1][3] = curCam.translation[1];
    tempCameraExtrinsic[2][3] = curCam.translation[2];
    curCam.extrinsics = tempCameraExtrinsic;
    glm::mat4 tempCameraIntrinsicH(0.0);
    tempCameraIntrinsicH[0][0] = curCam.intrinsics[0][0];
    tempCameraIntrinsicH[0][1] = curCam.intrinsics[0][1];
    tempCameraIntrinsicH[0][2] = curCam.intrinsics[0][2];
    tempCameraIntrinsicH[1][0] = curCam.intrinsics[1][0];
    tempCameraIntrinsicH[1][1] = curCam.intrinsics[1][1];
    tempCameraIntrinsicH[1][2] = curCam.intrinsics[1][2];
    tempCameraIntrinsicH[2][0] = curCam.intrinsics[2][0];
    tempCameraIntrinsicH[2][1] = curCam.intrinsics[2][1];
    tempCameraIntrinsicH[2][2] = curCam.intrinsics[2][2];

    curCam.cameraMatrix = tempCameraExtrinsic * tempCameraIntrinsicH;

//    std::cout<<"Counter" << i <<std::endl;
//    utilities::printMatrix("intrinsics", curCam.intrinsics);
//    utilities::printMatrix("cameraMatrix", curCam.cameraMatrix);
//    utilities::printMatrix("translation", curCam.translation);
//    utilities::printMatrix("rotation", curCam.rotation);
    cameras_.push_back(curCam);
    i++;
  }
}

bool comparePath(const boost::filesystem::path& path1, const boost::filesystem::path& path2) {
  int idx1, idx2;
  std::string cur1 = path1.filename().string();
  std::string cur2 = path2.filename().string();
  std::istringstream ss1(cur1.substr(0, 4));
  ss1 >> idx1;
  std::istringstream ss2(cur2.substr(0, 4));
  ss2 >> idx2;
  return idx1 < idx2;

}
void EpflParser::orderPaths(const boost::filesystem::path &path) {

  boost::filesystem::recursive_directory_iterator end;

  for (boost::filesystem::recursive_directory_iterator i(path); i != end; ++i) {
    const boost::filesystem::path cp = (*i);
    if (boost::filesystem::is_regular_file(cp) && cp.filename().string().length() == 15) {
      cameraPaths_.push_back(cp);
    }
  }

  std::sort(cameraPaths_.begin(), cameraPaths_.end(), comparePath);

}
} /* namespace reconstructorEvaluator */

