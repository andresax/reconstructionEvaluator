#ifndef INCLUDE_RECONSTRUCTIONEVALUATOR_DEPTHFROMPCL_H_
#define INCLUDE_RECONSTRUCTIONEVALUATOR_DEPTHFROMPCL_H_

#include <string>
#include <vector>
#include <glm.hpp>
#include <types.hpp>
#include <CImg/CImg.h>
namespace reconstructorEvaluator {

class DepthFromPCL {
public:
  DepthFromPCL();
  virtual ~DepthFromPCL();
  const cimg_library::CImg<float>& getDepth() const {
    return depth;
  }

  void run(std::string path, const CameraType& cam);
private:
  cimg_library::CImg<float> depth;
  bool loaded = false;
  std::vector<std::vector<float> > points;
};

} /* namespace reconstructorEvaluator */

#endif /* INCLUDE_RECONSTRUCTIONEVALUATOR_DEPTHFROMPCL_H_ */
