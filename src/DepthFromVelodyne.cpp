/*
 * DepthFromVelodyne.cpp
 *
 *  Created on: Dec 15, 2016
 *      Author: andrea
 */

#include <DepthFromVelodyne.h>
#include <boost/filesystem.hpp>
#include <utilities.hpp>
#include <CImg/CImg.h>

namespace reconstructorEvaluator {

DepthFromVelodyne::DepthFromVelodyne(const std::string &path, const int & imageHeight, const int & imageWidth) {

  pathBase_ = path;
  imageHeight_ = imageHeight;
  imageWidth_ = imageWidth;
  loadCalib();

}

DepthFromVelodyne::~DepthFromVelodyne() {
}

void DepthFromVelodyne::loadCalib() {

  std::string filename(pathBase_ + "/calib.txt");

  std::ifstream input(filename, std::ios::in);

  if (!input.good() || !boost::filesystem::is_regular_file(filename)) {
    std::cerr << ("LidarLoader", "Calibration Read: Could not read file: " + filename) << std::endl;

    input.close();
    return;
  }
  input.seekg(0, std::ios::beg);
  std::string line;

  std::string dummy;

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

    P.push_back(mat);
  }

  std::stringstream ss;
  glm::mat4 tr = glm::mat4(0.0);
  std::getline(input, line);
  ss << line;
  ss >> dummy;
  ss >> tr[0][0];
  ss >> tr[0][1];
  ss >> tr[0][2];
  ss >> tr[0][3];
  ss >> tr[1][0];
  ss >> tr[1][1];
  ss >> tr[1][2];
  ss >> tr[1][3];
  ss >> tr[2][0];
  ss >> tr[2][1];
  ss >> tr[2][2];
  ss >> tr[2][3];

  input.close();
}

void DepthFromVelodyne::createDepthFromIdx(int idx) {

  std::string filename(pathBase_ + "/velodyne/" + utilities::getFrameNumber(idx, 6) + ".bin");
  std::ifstream input(filename, std::ios::in | std::ios::binary);

  cimg_library::CImg<float> depth(imageWidth_, imageHeight_);
  depth.fill(-1.0);
  while (input.good() && !input.eof()) {
    float dummy;
    float point[3];
    input.read((char *) point, 3 * sizeof(float));
    input.read((char *) &dummy, sizeof(float));
    glm::vec4 pt3d = glm::vec4(point[0], point[1], point[2], 1.0);

    glm::mat4 mm = tr * P[0];
    glm::vec4 pt2dH = pt3d * mm;

    glm::vec2 pt2d = glm::vec2(pt2dH.x / pt2dH.z, pt2dH.y / pt2dH.z);

    float distance = glm::length(glm::vec3(point[0], point[1], point[2]));

    std::cout<<distance<<std::endl;
    int idX = static_cast<int>(pt2d.x);
    int idY = static_cast<int>(pt2d.y);
    if (0 < idX && idX < imageWidth_ && //
        0 < idY && idY < imageHeight_ && //
        distance > 0.0 && (distance < depth(idX, idX) || depth(idX, idX) < 0.0)) {

      depth(idY, idX) = distance;
    }

  }
  depth.save_ascii("depth.txt");
  depth.save_png("depth.png");

  input.close();
} /* namespace reconstructorEvaluator */

}
