#include <DTUParser.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <utilities.hpp>

namespace reconstructorEvaluator {

DTUParser::DTUParser() {

}

DTUParser::~DTUParser() {
}

void DTUParser::parse(const boost::filesystem::path& path) {
  orderPaths(boost::filesystem::path(path.string()), cameraPaths_);

  float fx = 2892.843;
  float fy = 2882.249;
  int w = 1600;
  int h = 1200;


  float cx = 824.4251;
  float cy = 605.18715;

  int i = 0;

  for (auto camPath : cameraPaths_) {

    CameraType curCam;
    std::ifstream camFile(camPath.string());

    curCam.intrinsics[0][0] = fx;
    curCam.intrinsics[1][1] = fy;
    curCam.intrinsics[0][2] = cx;
    curCam.intrinsics[1][2] = cy;
    curCam.intrinsics[2][2] = 1.0;

    curCam.cameraMatrix = glm::mat4(0.0);
    //Reading the transpose rotation  since the file stores the inverse of the Hartley Zisserman convention
    camFile >> curCam.cameraMatrix[0][0] >> curCam.cameraMatrix[0][1] >> curCam.cameraMatrix[0][2] >> curCam.cameraMatrix[0][3];
    camFile >> curCam.cameraMatrix[1][0] >> curCam.cameraMatrix[1][1] >> curCam.cameraMatrix[1][2] >> curCam.cameraMatrix[1][3];
    camFile >> curCam.cameraMatrix[2][0] >> curCam.cameraMatrix[2][1] >> curCam.cameraMatrix[2][2] >> curCam.cameraMatrix[2][3];

    glm::mat3 intrInv = glm::inverse(curCam.intrinsics);
    glm::mat4 Kinv = glm::mat4(0.0);
    for (int curR = 0; curR < 3; ++curR) {
      for (int curC = 0; curC < 3; ++curC) {
        Kinv[curR][curC] = intrInv[curR][curC];
      }
    }
    curCam.extrinsics = curCam.cameraMatrix * Kinv;
    curCam.extrinsics[3][3] = 1.0;

    for (int curR = 0; curR < 3; ++curR) {
      for (int curC = 0; curC < 3; ++curC) {
        curCam.rotation[curR][curC] = curCam.extrinsics[curR][curC];
      }
    }
    for (int curR = 0; curR < 3; ++curR) {
      curCam.translation[curR] = curCam.extrinsics[curR][3];
    }

    curCam.center = -curCam.translation * glm::transpose(curCam.rotation);

//    std::cout << "Counter" << i << std::endl;
//    utilities::printMatrix("intrinsics", curCam.intrinsics);
//    utilities::printMatrix("cameraMatrix", curCam.cameraMatrix);
//    utilities::printMatrix("translation", curCam.translation);
//    utilities::printMatrix("rotation", curCam.rotation);
//    utilities::printMatrix("center", curCam.center);

    curCam.imageHeight = h;
    curCam.imageWidth =w;
    cameras_.push_back(curCam);
    i++;
  }
}
bool comparePathDTU(const boost::filesystem::path& path1, const boost::filesystem::path& path2) {
  int idx1, idx2;
  std::string cur1 = path1.filename().string();
  std::string cur2 = path2.filename().string();
  std::istringstream ss1(cur1.substr(5, 3));

  ss1 >> idx1;
  std::istringstream ss2(cur2.substr(5, 3));
  ss2 >> idx2;
  return idx1 < idx2;

}

void DTUParser::orderPaths(const boost::filesystem::path &path,std::vector<boost::filesystem::path> &paths) {

  boost::filesystem::recursive_directory_iterator end;
  std::cout << path.string() << std::endl;
  for (boost::filesystem::recursive_directory_iterator i(path); i != end; ++i) {
    const boost::filesystem::path cp = (*i);
    if (boost::filesystem::is_regular_file(cp)) {
      paths.push_back(cp);
    }
  }

  std::sort(paths.begin(), paths.end(), comparePathDTU);
}

} /* namespace reconstructorEvaluator */
