#include <DepthFromPCL.h>
#include <fstream>

namespace reconstructorEvaluator {

DepthFromPCL::DepthFromPCL() {

}

DepthFromPCL::~DepthFromPCL() {
}

void DepthFromPCL::run(std::string path, const CameraType& cam) {

  std::ifstream input(path);
  std::string line;
  int w = cam.imageWidth;
  int h = cam.imageHeight;

  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);
  std::getline(input, line);

  std::vector<glm::vec3> prprp;
  depth = cimg_library::CImg<float>(w, h);
  depth.fill(-1.0);
  while (input.good() && !input.eof()) {
    std::string line;
    std::getline(input, line);

    float point[3];
    sscanf(line.c_str(), "%f %f %f", &point[0], &point[1], &point[2]);

    glm::vec4 pt2dH = glm::vec4(point[0], point[1], point[2], 1.0) * cam.cameraMatrix;
    glm::vec2 pt2d = glm::vec2(pt2dH.x / pt2dH.z, pt2dH.y / pt2dH.z);

    float distance = glm::length(cam.center-glm::vec3(point[0], point[1], point[2]));
    int idX = static_cast<int>(pt2d.x);
    int idY = static_cast<int>(pt2d.y);

    if (0 < idX && idX < w && //
        0 < idY && idY < h && //
        distance > 0.0 && (distance < depth(idX, idY) || depth(idX, idY) < 0.0)) {
      depth(idX, idY) = distance;

      glm::vec3 vecCenterpt2d = distance
          * glm::normalize(glm::vec3((idX - cam.intrinsics[0][2]) / cam.intrinsics[0][0], (idY - cam.intrinsics[1][2]) / cam.intrinsics[1][1], 1.0));

      prprp.push_back(glm::vec3(vecCenterpt2d.x, vecCenterpt2d.y, vecCenterpt2d.z));
    }

  }

}

} /* namespace reconstructorEvaluator */
