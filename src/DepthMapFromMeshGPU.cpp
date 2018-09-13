#include <DepthMapFromMeshGPU.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <fstream>
#include <DepthMapXYZProgram.h>
#include <DepthMapProgram.h>
#include <iostream>
#include <utilities.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace reconstructorEvaluator {

typedef Polyhedron::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::FT FT;
typedef Kernel::Segment_3 Segment;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

DepthMapFromMeshGPU::DepthMapFromMeshGPU(Polyhedron *mesh, int imageWidth, int imageHeight) {
  mesh_ = mesh;
  imageHeight_ = imageHeight;
  imageWidth_ = imageWidth;
  initialize();
}

void DepthMapFromMeshGPU::computeMap(const CameraType& cam,int num,float scale_) {
  // std::cout <<std::endl << "MVP..."<<std::endl;
  // utilities::printMatrix(cam.mvp);
  // std::cout <<std::endl << "DONE"<<std::endl;

  // std::cout <<std::endl << "Center..."<<std::endl;
  // utilities::printMatrix(cam.center);
  // std::cout <<std::endl << "DONE"<<std::endl;

  depth = cimg_library::CImg<float>(cam.imageWidth, cam.imageHeight);
  depth.fill(-1.0);

  // std::cout <<std::endl << "compute depth init..."<<std::flush;
  depthProgram_->setArrayBufferObj(vertexBufferObj_, mesh_->size_of_facets() * 3);
  depthProgram_->setUseElementsIndices(false);
  static_cast<DepthMapProgram *>(depthProgram_)->computeDepthMap(framebufferDepth_, cam.mvp);
  glFinish();
  // std::cout<<"DONE"<<std::endl;


  // std::cout << "compute depthXYZ init..."<<std::flush;
  //************************reprojection**************************
  depthXYZProgram_->setArrayBufferObj(vertexBufferObj_, mesh_->size_of_facets() * 3);
  depthXYZProgram_->setUseElementsIndices(false);
  static_cast<DepthMapXYZProgram *>(depthXYZProgram_)->setCenter(cam.center);
  static_cast<DepthMapXYZProgram *>(depthXYZProgram_)->setDepthTexture(depthTexture_);
  static_cast<DepthMapXYZProgram *>(depthXYZProgram_)->setMvp(cam.mvp);
  depthXYZProgram_->compute(true);
  glFinish();
  // std::cout<<"DONE"<<std::endl;
  
  SwapBuffers();
  cv::Mat depthMat;
  CaptureViewPortFloatToFloat(depthMat, GL_RGB, 3,static_cast<DepthMapXYZProgram *>(depthXYZProgram_)->getFramebuffer());
  // sleep(1.0);  
  // std::cout<<"CaptureViewPortFloat DONE"<<std::endl;


  std::vector<glm::vec3> prprp;
  for (int col = 0; col < depthMat.cols; ++col) {
    for (int row = 0; row < depthMat.rows; ++row) {

      float scale;//=0.9*0.50354;     scale=scale_*1.05;      scale=1.0f;
      scale=1-0.532745;//0095
      scale=0.53;//0104
      
      float distance = scale * depthMat.at<cv::Vec3f>(row, col)[2]+scale * depthMat.at<cv::Vec3f>(row, col)[0]+scale * depthMat.at<cv::Vec3f>(row, col)[1];

      if (distance < 15.0 && distance > 0.0 ) {

         float cx = 609.5593;
        // float cy = 210.188;
        // float cx = 612.0;
        // float cy = 160.1;
         float cy =172.854;
        // float fx =758.88;
        //float cx = cam.intrinsics[0][2];
        //float cy = cam.intrinsics[1][2];
           // float cy =172.854;
        float fx =721.0;
        float fy =721.0;
        // float fx =cam.intrinsics[0][0];
        // float fy =cam.intrinsics[1][1];
        glm::vec3 vecCenterpt2d = glm::normalize(glm::vec3((col - cx)/fx , (row - cy)/fy , 1.0));
        prprp.push_back(distance*vecCenterpt2d);
        depth(col, row) = distance;
      }
    }
  }
 //  if(num>200){
 // std::ofstream file("outMesh2.ply");
 //   file<<"ply"<<std::endl<<"format ascii 1.0"<<std::endl<<"element vertex " <<prprp.size() <<std::endl<<"property float x"
 //       <<std::endl<<"property float y"<<std::endl<<"property float z"<<std::endl<<" end_header"<<std::endl;
 // for(auto dd:prprp){
 //   file<< dd.x<<" "<<dd.y<<" "<<dd.z<<std::endl;
 // }
 // file.close();
 // }

 // depth.save_ascii("depthMesh.txt");

//  cimg_library::CImg<float> deptht=depth;
//  deptht.normalize(0, 255);
//  deptht.save_png("depthMesh.png");

}

void DepthMapFromMeshGPU::initialize(){


  vertexBufferObj_  = framebufferDepth_ = depthTexture_ = depthXYZTexture_ =-1;

  depthProgram_ = new DepthMapProgram(imageWidth_, imageHeight_);
  depthXYZProgram_ = new DepthMapXYZProgram(imageWidth_, imageHeight_);

  init();
  createVertexArrayBuffer();
    //************************depth********************************
  std::cout << "DepthShaderProgram init...";
  depthProgram_->initializeProgram();
  static_cast<DepthMapProgram *>(depthProgram_)->initializeFramebufAndTex(framebufferDepth_, depthTexture_);
  std::cout << "DONE" << std::endl;

  //************************reprojection**************************
  std::cout << "DepthShaderProgramXYZ init...";
  depthXYZProgram_->initializeProgram();
  static_cast<DepthMapXYZProgram *>(depthXYZProgram_)->initializeFramebufAndTex(depthXYZTexture_);
  std::cout << "DONE" << std::endl;
}

void DepthMapFromMeshGPU::resetVertexArrayBuffer() {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObj_);
  std::vector<glm::vec3> verticesUnwrapped;

  for (Polyhedron::Facet_iterator itFac = mesh_->facets_begin(); itFac != mesh_->facets_end(); itFac++) {
    Polyhedron::Halfedge_handle h0, h1, h2;
    h0 = itFac->halfedge();
    h1 = h0->next();
    h2 = h1->next();

    Polyhedron::Vertex_handle v0, v1, v2;
    v0 = h0->vertex();
    v1 = h1->vertex();
    v2 = h2->vertex();

    verticesUnwrapped.push_back(glm::vec3(v0->point().x(), v0->point().y(), v0->point().z()));
    verticesUnwrapped.push_back(glm::vec3(v1->point().x(), v1->point().y(), v1->point().z()));
    verticesUnwrapped.push_back(glm::vec3(v2->point().x(), v2->point().y(), v2->point().z()));
  }
  glBufferData(GL_ARRAY_BUFFER, 3 * mesh_->size_of_facets() * sizeof(glm::vec3), &verticesUnwrapped[0], GL_STATIC_DRAW);

  std::cout<<"MeshViewer::resetVertexArrayBuffer verticesUnwrapped.size()="<<verticesUnwrapped.size()<<std::endl;
}


void DepthMapFromMeshGPU::createVertexArrayBuffer() {
  glGenBuffers(1, &vertexBufferObj_);
  resetVertexArrayBuffer();
}

} /* namespace reconstructorEvaluator */
