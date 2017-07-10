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

  rawSeq_ = false;
  odoSeq_ = false;
  pathBase_ = path;
  imageHeight_ = imageHeight;
  imageWidth_ = imageWidth;
  loadCalib();

}

void DepthFromVelodyne::loadCalib() {

  std::string filename;
  filename =(pathBase_ + "/calib.txt");
  std::ifstream input;

  input.open(filename.c_str(), std::ios::in);
  if (!input.good() || !boost::filesystem::is_regular_file(filename)) {
    filename = (pathBase_ + "/calib_cam_to_cam.txt");
    input.open(filename.c_str(), std::ios::in);
    if (input.good() && boost::filesystem::is_regular_file(filename)) {
      rawSeq_ = true;
    } else {
      std::cerr << ("LidarLoader", "Calibration Read: Could not read file: " + filename) << std::endl;
      input.close();
      return;
    }
  } else {
    odoSeq_ = true;
  }


  if (odoSeq_ == true) {
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
    tr = glm::mat4(0.0);
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
  }

  if (rawSeq_==true) {
    std::string line;
    std::string dummy;
    std::stringstream ss;
    glm::mat4 tmp;

    std::getline(input, line); //calib_time:
    std::getline(input, line); //corner_dist
    std::getline(input, line); //S_00
    std::getline(input, line); //K_00
    std::getline(input, line); //D_00
    std::getline(input, line); //R_00
    std::getline(input, line); //T_00
    std::getline(input, line); //S_rect_00
    std::getline(input, line); //R_rect_00
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2];
    R_rect.push_back(tmp);
    std::getline(input, line); //P_rect_00
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[0][3] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[1][3] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2]
        >> tmp[2][3];
    P.push_back(tmp);
    std::getline(input, line); //S_01
    std::getline(input, line); //K_01
    std::getline(input, line); //D_01
    std::getline(input, line); //R_01
    std::getline(input, line); //T_01
    std::getline(input, line); //S_rect_01
    std::getline(input, line); //R_rect_01
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2];
    R_rect.push_back(tmp);
    std::getline(input, line); //P_rect_01
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[0][3] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[1][3] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2]
        >> tmp[2][3];
    P.push_back(tmp);
    std::getline(input, line); //S_02
    std::getline(input, line); //K_02
    std::getline(input, line); //D_02
    std::getline(input, line); //R_02
    std::getline(input, line); //T_02
    std::getline(input, line); //S_rect_02
    std::getline(input, line); //R_rect_02
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2];
    R_rect.push_back(tmp);
    std::getline(input, line); //P_rect_02
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[0][3] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[1][3] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2]
        >> tmp[2][3];
    P.push_back(tmp);
    std::getline(input, line); //S_03
    std::getline(input, line); //K_03
    std::getline(input, line); //D_03
    std::getline(input, line); //R_03
    std::getline(input, line); //T_03
    std::getline(input, line); //S_rect_03
    std::getline(input, line); //R_rect_03
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2];
    R_rect.push_back(tmp);
    std::getline(input, line); //P_rect_03
    tmp = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[0][3] >> tmp[1][0] >> tmp[1][1] >> tmp[1][2] >> tmp[1][3] >> tmp[2][0] >> tmp[2][1] >> tmp[2][2]
        >> tmp[2][3];
    P.push_back(tmp);

    std::string filename2(pathBase_ + "/calib_velo_to_cam.txt");
    std::ifstream input2(filename2, std::ios::in);
    std::getline(input2, line); //calib_time
    std::getline(input2, line); //R
    E_velo_to_cam = glm::mat4(0.0);
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> E_velo_to_cam[0][0] >> E_velo_to_cam[0][1] >> E_velo_to_cam[0][2] >> //
        E_velo_to_cam[1][0] >> E_velo_to_cam[1][1] >> E_velo_to_cam[1][2] //
        >> E_velo_to_cam[2][0] >> E_velo_to_cam[2][1] >> E_velo_to_cam[2][2];

    std::getline(input2, line); //T
    ss.clear();
    ss.str("");
    ss << line;
    ss >> dummy >> E_velo_to_cam[0][3] >> E_velo_to_cam[1][3] >> E_velo_to_cam[2][3];

    input2.close();
  }

  input.close();
}

void DepthFromVelodyne::createDepthFromIdx(int idx) {

  std::string filename;
  if (odoSeq_==true) {
    filename = (pathBase_ + "/velodyne/" + utilities::getFrameNumber(idx, 6) + ".bin");
  }
  if (rawSeq_==true) {
    filename = (pathBase_ + "/velodyne_points/data/" + utilities::getFrameNumber(idx / 2, 10) + ".bin");
  }

  std::ifstream input(filename, std::ios::in | std::ios::binary);

  std::ofstream file("velo2.txt");

  depth = cimg_library::CImg<float>(imageWidth_, imageHeight_);
  depth.fill(-1.0);
  while (input.good() && !input.eof()) {
    float dummy;
    float point[3];
    input.read((char *) point, 3 * sizeof(float));
    input.read((char *) &dummy, sizeof(float));
    if (point[0] > 0.0) {
      glm::vec2 pt2d;
      glm::vec4 pt3,pt2evwd, pt2dH;
      glm::vec4 pt3d = glm::vec4(point[0], point[1], point[2], 1.0);
      if (odoSeq_==true) {
        pt2dH = pt3d * tr * P[0];
        pt3 = pt3d * tr;
      }
      if (rawSeq_==true) {
        pt2dH = pt3d * E_velo_to_cam * R_rect[2] * P[2];
//        utilities::printMatrix("E_velo_to_cam",E_velo_to_cam);
//        utilities::printMatrix("pt3d * E_velo_to_cam",pt3d * E_velo_to_cam);
//        utilities::printMatrix("pt3d",pt3d);
//        exit(0);
        pt3 = pt3d;
      }
      file << pt3.x << " " << pt3.y << " " << pt3.z << " " << std::endl;
      pt2d = glm::vec2(pt2dH.x / pt2dH.z, pt2dH.y / pt2dH.z);
      float distance = glm::length(glm::vec3(point[0], point[1], point[2]));

      int idX = static_cast<int>(pt2d.x);
      int idY = static_cast<int>(pt2d.y);

      if (distance < 30.0 && 0 < idX && idX < imageWidth_ && //
          0 < idY && idY < imageHeight_ && //
          distance > 0.0 && (distance < depth(idX, idY) || depth(idX, idY) < 0.0)) {
        depth(idX, idY) = distance;
      }
    }

  }
  file.close();
//  depth.save_ascii("depth.txt");
  depth.save_png("depth.png");

  input.close();
//  exit(0);
} /* namespace reconstructorEvaluator */

}
