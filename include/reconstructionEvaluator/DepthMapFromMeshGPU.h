
#ifndef SRC_DEPTHMAPFROMMESHGPU_H_
#define SRC_DEPTHMAPFROMMESHGPU_H_

#include <types.hpp>
#include <CImg/CImg.h>
#include <OpenGLProgram.h>
#include <ShaderProgram.h>
namespace reconstructorEvaluator {

class DepthMapFromMeshGPU: public openGLUtilities::OpenGLProgram {
public:
  DepthMapFromMeshGPU(Polyhedron *mesh, int imageWidth, int imageHeight);
  virtual ~DepthMapFromMeshGPU() = default;

  void setMeshGt(Polyhedron*& mesh) {
    mesh_ = mesh;
  }

  void computeMap(const CameraType &cam,int nnum=0,float scale=1.0);

  const cimg_library::CImg<float>& getDepth() const {
    return depth;
  }

private:
  void createVertexArrayBuffer();
  void resetVertexArrayBuffer();
  void initialize();

  openGLUtilities::ShaderProgram *depthProgram_;
  openGLUtilities::ShaderProgram *depthXYZProgram_;
  GLuint depthTexture_,vertexBufferObj_,framebufferDepth_,depthXYZTexture_;

  Polyhedron *mesh_;
  std::vector<CameraType> cams_;
  cimg_library::CImg<float> depth;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_DEPTHMAPFROMMESHGPU_H_ */
