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
#include <DTUParser.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace reconstructorEvaluator {

CameraParser::CameraParser() {
}

CameraParser::~CameraParser() {
}

bool CameraParser::parseCameras(const boost::filesystem::path &path) {

  std::cout << "parsing..." << path.string() << std::endl;
  if (boost::filesystem::exists(path)) {
    if (boost::filesystem::is_directory(path)) {
      boost::filesystem::path::iterator it = path.begin();
      if (path.parent_path().filename().string() == "camerasKE") {
        EpflParser epflParser;
        epflParser.parse(path);
        cameras_ = epflParser.getCameras();

      } else if (path.parent_path().filename().string() == "calib") {
        DTUParser dtuParser;
        dtuParser.parse(path);
        cameras_ = dtuParser.getCameras();

      } else {
        std::cout << "CameraParser::parseCameras ERROR: path is a directory but it does not link to a CamerasKE folder";
        std::cout << ";  " << path.parent_path().filename().string() << " instead" << std::endl;
        return false;

      }
    } else if (boost::filesystem::is_regular_file(path)) {
      if (boost::filesystem::extension(path) == ".json") {
        OpenMvgParser ompars;
        ompars.parse(path);
        cameras_ = ompars.getCameras();

      } else if (boost::filesystem::extension(path) == ".txt") {
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


  for (int curCam = 0; curCam < cameras_.size(); curCam++) {
    convertToMvp2(cameras_[curCam],cameras_[curCam].mvp);
  }


  return true;

}

void CameraParser::convertToMvp2(CameraType &cam, glm::mat4 &mvpOut) {

  glm::mat4 modelViewMatrix, projectionMatrix;
   computeModelViewMatrix2(cam.rotation, cam.translation, modelViewMatrix);
  computeProjectionMatrix2(cam.intrinsics, cam.imageHeight, cam.imageWidth, projectionMatrix);
  mvpOut = projectionMatrix * modelViewMatrix;
}

void CameraParser::computeProjectionMatrix2(glm::mat3 &intrinsics, int h, int w, glm::mat4 &projectionMatrixOut) {

  float N = 0.1;
  float F = 500.0;

  glm::mat4 persp = glm::mat4(0.0);
  persp[0][0] = intrinsics[0][0];
  persp[0][1] = 0.0;
  persp[0][2] = intrinsics[0][2];
  persp[1][1] = intrinsics[1][1];
  persp[1][2] = intrinsics[1][2];
  persp[2][2] = -(N + F);
  persp[2][3] = N * F;
  persp[3][2] = 1.0;

  double L = 0;
  double R = w;
  double B = 0;
  double T = h;
  glm::mat4 ortho = glm::mat4(0.0);

  int dino = 1;/*to change the y sign if dino file format is used*/
  ortho[0][0] = 2.0 / (R - L);
  ortho[0][3] = -(R + L) / (R - L);
  ortho[1][1] = (1 - 2 * dino) * 2.0 / (T - B);
  ortho[1][3] = -(1 - 2 * dino) * (T + B) / (T - B);
  ortho[2][2] = (-2.0 / (F - N));
  ortho[2][3] = (-(F + N) / (F - N));
  ortho[3][3] = 1.0;
  projectionMatrixOut = glm::transpose(persp * ortho);

}

void CameraParser::computeModelViewMatrix2(glm::mat3 &rotation, glm::vec3 &translation, glm::mat4 &modelViewMatrixOut) {

  glm::mat4 modelMatrix = glm::mat4();
  glm::mat4 viewMatrix = glm::mat4();

  viewMatrix[0][0] = rotation[0][0];
  viewMatrix[0][1] = rotation[1][0];
  viewMatrix[0][2] = rotation[2][0];
  viewMatrix[1][0] = rotation[0][1];
  viewMatrix[1][1] = rotation[1][1];
  viewMatrix[1][2] = rotation[2][1];
  viewMatrix[2][0] = rotation[0][2];
  viewMatrix[2][1] = rotation[1][2];
  viewMatrix[2][2] = rotation[2][2];
  viewMatrix[3][0] = translation.x;
  viewMatrix[3][1] = translation.y;
  viewMatrix[3][2] = translation.z;
  viewMatrix[0][3] = 0.0;
  viewMatrix[1][3] = 0.0;
  viewMatrix[2][3] = 0.0;
  viewMatrix[3][3] = 1.0;

  modelViewMatrixOut = viewMatrix * modelMatrix;
}


void CameraParser::reset() {
  cameras_.clear();
}
} /* namespace reconstructorEvaluator */

