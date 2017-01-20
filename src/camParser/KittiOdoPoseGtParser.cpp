/*
 * KittiOdoPoseGtParser.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: andrea
 */

#include <KittiOdoPoseGtParser.h>
#include <fstream>
#include <iostream>

namespace reconstructorEvaluator {

KittiOdoPoseGtParser::KittiOdoPoseGtParser() {
w=1240,h=376;
}

KittiOdoPoseGtParser::~KittiOdoPoseGtParser() {
}

void KittiOdoPoseGtParser::parse(const boost::filesystem::path& path) {
  std::ifstream fileIn(path.string());
  std::cout<<"reading: "<<path.string()<<std::endl;
  CameraType curCam;
  glm::mat4 temp;
  while (!fileIn.eof()) {
    bool err = false;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 4; col++) {
        if (!(fileIn.eof())) {
          fileIn >> temp[row][col];
//          std::cout << curCam.extrinsics[row][col] << " ";
        }
      }
    }
//    std::cout <<std::endl;
    if (!err) {
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          curCam.rotation[row][col] = temp[col][row];
          curCam.extrinsics[row][col] = curCam.rotation[row][col];
        }
      }
      for (int row = 0; row < 3; row++) {
        curCam.center[row] = temp[row][3];
      }
      curCam.translation = -curCam.center * (curCam.rotation);

      for (int row = 0; row < 3; row++) {
        curCam.extrinsics[row][3] = curCam.translation[row];
      }

      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          curCam.intrinsics[row][col] = intrinsics_[row][col];
        }
      }
      curCam.imageHeight = h;
      curCam.imageWidth = w;
      curCam.cameraMatrix = curCam.extrinsics * intrinsics_;
      cameras_.push_back(curCam);
    }

  }

}

void KittiOdoPoseGtParser::setPathIntrinsicCalib(const boost::filesystem::path& path, int i) {
  if (i > 3 || i < 0) {
    std::cout << "setPathIntrinsicCalib i must be 0<i< 4" << std::endl;
    intrinsics_ = glm::mat4(1.0);
  }
  std::string filename(path.string() + "/calib.txt");

  std::ifstream input(filename, std::ios::in);
  std::vector<glm::mat4> intr;
  std::string line;
  std::string dummy;

  if (!input.good() || !boost::filesystem::is_regular_file(filename)) {
    std::cerr << ("LidarLoader", "Calibration Read: Could not read file: " + filename) << std::endl;

    input.close();
    return;
  }
  input.seekg(0, std::ios::beg);

  for (int curCam = 0; curCam < 4; ++curCam) {
    glm::mat4 mat = glm::mat4(0.0);
    std::stringstream ss;

    //The second line is the rotation matrix... Parse it
    std::getline(input, line);
    ss << line;
    //Drop the "R:" from the beginning of the line
    ss >> dummy;
    //Parse the doubles..
    ss >> mat[0][0];
    ss >> mat[0][1];
    ss >> mat[0][2];
    ss >> mat[0][3];
    ss >> mat[1][0];
    ss >> mat[1][1];
    ss >> mat[1][2];
    ss >> mat[1][3];
    ss >> mat[2][0];
    ss >> mat[2][1];
    ss >> mat[2][2];
    ss >> mat[2][3];

    intr.push_back(mat);
  }

  intrinsics_ = intr[i];

  input.close();
}

} /* namespace reconstructorEvaluator */
