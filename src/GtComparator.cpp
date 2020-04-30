/*
 * GtComparator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <GtComparator.h>
#include <Polyhedron_AssimpToCgal.h>
#include <assimp/Importer.hpp>  // OO version Header!
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <DepthMapFromMesh.h>
#include <DepthMapFromMeshGPU.h>
#include <DepthFromVelodyne.h>
#include <DepthFromPCL.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <utilities.hpp>
#include <CImg/CImg.h>
#include <opencv2/core.hpp>
#include <CGAL/IO/Polyhedron_iostream.h>

namespace reconstructorEvaluator {

typedef Polyhedron::HalfedgeDS HalfedgeDS;
GtComparator::GtComparator(const std::string& path) {
  configuration_.setConfiguration(path);
  configuration_.parse();
  scale_ = 1.0;
  countImages_ = 0;
}

GtComparator::~GtComparator() {

}

void GtComparator::run2() {

  std::ifstream file(configuration_.getMeshPath());
  file >> meshToBeCompared_;
  cimg_library::CImg<float> depthGT;
  for (int curFrame = configuration_.getInitFrame(); curFrame < configuration_.getLastFrame(); (configuration_.isStereo() ? curFrame += 5 : ++curFrame)) {

    std::cout << "GtComparator:: collecting errors frame num " << curFrame << std::endl;
    //std::cout << "loading gt... " << std::flush;
    //loadDepthMapGT(configuration_.getGtPath(), depthGT, configuration_.getCameras()[curFrame].imageWidth,configuration_.getCameras()[curFrame].imageHeight);
    //std::cout << "DONE" << std::endl;
  //  exit(0);

    DepthMapFromMesh dmfm(&meshToBeCompared_);
    dmfm.computeMap(configuration_.getCameras()[curFrame], curFrame);
    DepthFromPCL sfpcl;
    sfpcl.run(configuration_.getGtPath(), configuration_.getCameras()[curFrame]);
    depthGT = sfpcl.getDepth();
    accumulateDepthMaps(depthGT, dmfm.getDepth());
    countImages_++;
    std::cout << "DONE." << std::endl;
//    compareDepthMaps();
//    printComparison();
//    exit(0);
  }
  compareDepthMaps();
  printComparison();
}


void GtComparator::run() {
  // std::ifstream file(configuration_.getMeshPath());
  // file >> meshToBeCompared_;
  for(int curFrame = configuration_.getInitFrame(); curFrame <= configuration_.getLastFrame();curFrame++){
  cimg_library::CImg<float> depthGT;
  // int curFrame = configuration_.getInitFrame();
  std::cout << "GtComparator:: collecting errors frame num " << curFrame << std::endl;
  // std::cout << "loading gt... " << std::flush;
  // loadDepthMapGT(configuration_.getGtPath(), depthGT, configuration_.getCameras()[curFrame].imageWidth,configuration_.getCameras()[curFrame].imageHeight);
  // std::cout << "DONE" << std::endl;
  // depthGT.save_png("depthGT.png");
//  exit(0);

  if(configuration_.isGtSplitted()){

    std::cout<<"Multiple Mesh Mode"<<std::endl;
    cimg_library::CImg<float> depth;
    for (int i = 0; i < configuration_.getGtPaths().size(); ++i){
  Polyhedron meshGtCur;
      std::ifstream file(configuration_.getGtPaths()[i]);
      file >> meshGtCur;
      file.close();
      std::cout<<configuration_.getGtPaths()[i] <<" "<<meshGtCur.size_of_vertices()<<std::endl;

      // DepthMapFromMesh sfpcl(&meshGt_);
      DepthMapFromMeshGPU sfpcl(&meshGtCur,configuration_.getCameras()[0].imageWidth, configuration_.getCameras()[0].imageHeight);
      sfpcl.computeMap(configuration_.getCameras()[curFrame], curFrame);


      if (i==0) {
        depthGT = sfpcl.getDepth();
        depthGT.fill(-1.0);
      }

      cimg_library::CImg<float> depthGTCur;
      depthGTCur = sfpcl.getDepth();

      for (int col = 0; col < configuration_.getCameras()[curFrame].imageWidth; ++col) 
        for (int row = 0; row < configuration_.getCameras()[curFrame].imageHeight; ++row) 
          if (depthGTCur(col, row) > 0.0 && (depthGTCur(col, row) < depthGT(col, row) || depthGT(col, row) <0.0))
            depthGT(col, row) = depthGTCur(col, row);
      
    }
         
  }else{

    std::ifstream file2(configuration_.getGtPath());
    file2 >> meshGt_;

    std::cout<<"One Mesh Mode"<<std::endl;
    std::cout<<configuration_.getGtPath()<<" "<<meshGt_.size_of_vertices()<<std::endl;
      // DepthMapFromMesh sfpcl(&meshGt_);
    DepthMapFromMeshGPU sfpcl(&meshGt_,configuration_.getCameras()[0].imageWidth, configuration_.getCameras()[0].imageHeight);
    sfpcl.computeMap(configuration_.getCameras()[curFrame], curFrame);
    depthGT = sfpcl.getDepth();
  }

std::stringstream ss,ss1;
ss<<"depthMapFloatGT"<< curFrame<<".txt";
  std::ofstream fileOut(ss.str());
  float minD = -1,maxD=0;
  for (int j = 0; j < depthGT.height(); j++) {
    for (int i = 0; i < depthGT.width(); i++) {
        fileOut << " " << depthGT(i, j);
      if( depthGT(i, j) > 0.0){
        minD = (minD<0||minD>depthGT(i,j))?depthGT(i,j):minD;
        maxD = (maxD<depthGT(i,j))?depthGT(i,j):maxD;
      }
    }
    fileOut << std::endl;
  }
  fileOut.close();

  cimg_library::CImg<unsigned char> depthGT8U(depthGT.width(), depthGT.height());
  for (int i = 0; i < depthGT8U.width(); i++) {
    for (int j = 0; j < depthGT8U.height(); j++) {
        depthGT8U(i,j) = 255.0*(depthGT(i, j) - minD) / (maxD-minD);
    }
  }
ss1<<"depthGTFountain"<< curFrame<<".png";
const std::string filename = ss1.str();
std::cout<<filename<<std::endl;
 depthGT8U.save_png(filename.c_str());


}
exit(0);
  // // DepthMapFromMesh dmfm(&meshToBeCompared_);
  // // dmfm.computeMap(configuration_.getCameras()[curFrame], curFrame);
  // DepthFromPCL sfpcl;
  // sfpcl.run("/home/andrea/workspaceC/incrementalMVS/FountainInitOK.off", configuration_.getCameras()[curFrame]);
  // // sfpcl.run(configuration_.getGtPath(), configuration_.getCameras()[curFrame]);
  // depthGT.save_png("depthGT.png");
  // //sfpcl.save_png("depthGT.png");
  
  // //dmfm.getDepth().save_png("dmfm.png");
  // sfpcl.getDepth().save_png("dmfm.png");
  // accumulateDepthMaps(depthGT, sfpcl.getDepth());
  // countImages_++;
  // std::cout << "DONE." << std::endl;

  // compareDepthMaps();
  // printComparison();
}

void GtComparator::run3() {
  //registerCameras();
    std::cout << "configuration_.getMeshPath() " << configuration_.getMeshPath() << std::endl;
  std::ifstream file(configuration_.getMeshPath());
  file >> meshToBeCompared_;

  DepthMapFromMeshGPU dmfm(&meshToBeCompared_,configuration_.getCameras()[0].imageWidth, configuration_.getCameras()[0].imageHeight);
  DepthFromVelodyne frv(configuration_.getGtPath(), configuration_.getCameras()[0].imageHeight, configuration_.getCameras()[0].imageWidth);
  glm::mat3 intrinsics = configuration_.getCameras()[0].intrinsics;
  frv.setCalibCam(intrinsics[0][2], intrinsics[1][2], intrinsics[0][0], intrinsics[1][1]);

  // std::cout<<intrinsics[0][2]<<std::endl;
  // std::cout<<intrinsics[1][2]<<std::endl;
  // std::cout<<intrinsics[0][0]<<std::endl;
  // std::cout<<intrinsics[1][1]<<std::endl;
  // exit(0);

  for (int curFrame = configuration_.getInitFrame(); curFrame < configuration_.getLastFrame(); (configuration_.isStereo() ? curFrame += 6 : ++curFrame)) {
    std::cout << "GtComparator:: collecting errors frame num " << curFrame << std::flush;
    // if (configuration_.isStereo()) {
    //   float curbaseline = glm::length(configuration_.getCameras()[curFrame].center - configuration_.getCameras()[curFrame +1].center);
    //   scale_ = configuration_.getBaseline() / curbaseline;
    // }
    frv.createDepthFromIdx(curFrame);

    // DepthMapFromMesh dmfm(&meshToBeCompared_);
    dmfm.computeMap(configuration_.getCameras()[curFrame],curFrame);
// if(curFrame>200){
//    dmfm.getDepth().save_png("depth.png");
//    frv.getDepth().save_png("depthGT.png");
//  // 
     exit(0);
  // }
    accumulateDepthMaps(frv.getDepth(), dmfm.getDepth());
    countImages_++;
    std::cout << "DONE." << std::endl;
  }

  compareDepthMaps();
  printComparison();
}

void GtComparator::importGT() {

  Assimp::Importer importer;

  std::cout << "GtComparator::importGT importing...";
  std::cout.flush();
  const aiScene* scene = importer.ReadFile(configuration_.getGtPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  std::cout << "DONE." << std::endl;

  aiMesh* mesh = scene->mMeshes[0];

  std::cout << "GtComparator::poly_builderGT importing...";
  std::cout.flush();
  MeshBuilder<HalfedgeDS> poly_builder(mesh);
  meshGt_.delegate(poly_builder);
  std::cout << "DONE." << std::endl;

}

void GtComparator::importMesh() {

  Assimp::Importer importer;

  std::cout << "GtComparator::importMesh importing...";
  std::cout.flush();
  const aiScene* scene = importer.ReadFile(configuration_.getMeshPath(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  std::cout << "DONE." << std::endl;

  aiMesh* mesh = scene->mMeshes[0];

  std::cout << "GtComparator::poly_builder importing...";
  std::cout.flush();
  MeshBuilder<HalfedgeDS> poly_builder(mesh);
  meshToBeCompared_.delegate(poly_builder);
  std::cout << "DONE." << std::endl;
}

void GtComparator::compareDepthMaps(const std::vector<cimg_library::CImg<float>> &depthGTVec, const std::vector<cimg_library::CImg<float>> &depthVec) {
  if (depthGTVec.size() != depthVec.size()) {
    std::cout << " compareDepthMaps error the two vectors have different dimensions" << std::endl;
    return;
  }

  for (int curFrame = 0; curFrame < depthGTVec.size(); ++curFrame) {
    cimg_library::CImg<float> depthGT = depthGTVec[curFrame];
    cimg_library::CImg<float> depth = depthVec[curFrame];
    accumulateDepthMaps(depthGT, depth);

  }
  compareDepthMaps();
}

void GtComparator::compareDepthMaps() {

  float sum = std::accumulate(res.errs_.begin(), res.errs_.end(), 0.0);

  std::vector<float> sqrVec;
  std::transform(res.errs_.begin(), res.errs_.end(), std::back_inserter(sqrVec), [](float n) {return std::fabs(std::pow(n,2));});
  float sumSqr = std::accumulate(sqrVec.begin(), sqrVec.end(), 0.0);
  std::vector<float> absVec;
  std::transform(res.errs_.begin(), res.errs_.end(), std::back_inserter(absVec), [](float n) {return std::fabs(n);});
  float sumAbs = std::accumulate(absVec.begin(), absVec.end(), 0.0);

  res.mean = sum / res.errs_.size();
  // std::cout<<"RMSE CAZZO "<<sum<<" "<<res.errs_.size()<<std::endl;
  // std::cout<<"RMSE CAZZO "<<sumSqr<<" "<<res.errs_.size()<<std::endl;
  // std::cout<<"RMSE CAZZO "<<sumAbs<<" "<<res.errs_.size()<<std::endl;
  res.rmse = std::sqrt(sumSqr / res.errs_.size());
  res.mae = sumAbs / res.errs_.size();

  std::vector<double> diff(res.errs_.size());
  std::transform(res.errs_.begin(), res.errs_.end(), diff.begin(), std::bind2nd(std::minus<double>(), res.mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  res.stddev = std::sqrt(sq_sum / res.errs_.size());
}

void GtComparator::accumulateDepthMaps(const cimg_library::CImg<float>& depthGT, const cimg_library::CImg<float>& depth) {

  cimg_library::CImg<float> depthErrrr = cimg_library::CImg<float>(depthGT.width(), depth.height());

  depthErrrr.fill(-1.0);
  if (depthGT._width != depth._width || depthGT._height != depth._height) {
    std::cout << " compareDepthMaps error the two depth maps have different dimensions" << std::endl;
    return;
  }

  float XtX = 0.0;
  float Xty = 0.0;
  for (int x = 0; x < depthGT._width; ++x) {
    for (int y = 0; y < depthGT._height; ++y) {
      if (depth(x, y) > 0.0 && depthGT(x, y) > 0.0 && std::fabs(depth(x, y) - depthGT(x, y)) < 20.0) {
        XtX = XtX + (depth(x, y) * depth(x, y));
        Xty = Xty + (depth(x, y) * depthGT(x, y));
      }
    }
  }

  float curScale = Xty / XtX;
  curScale=1.0;

  std::cout << "SCALE " << Xty / XtX << std::endl;


  std::cout << std::endl << "SCALE OK " << scale_ << std::endl;
  for (int x = 0; x < depthGT._width; ++x) {
    for (int y = 0; y < depthGT._height; ++y) {
      if (depth(x, y) > 0.0 && depthGT(x, y) > 0.0 && std::fabs(curScale*depth(x, y) - depthGT(x, y)) < 2.0) {
        res.errs_.push_back((curScale * depth(x, y) - depthGT(x, y)));

        depthErrrr(x, y) = std::fabs((curScale * depth(x, y) - depthGT(x, y)));
      }
    }
  }
 depthErrrr.normalize(0, 255);
 depthErrrr.save_png("err.png");
}

void GtComparator::registerCameras() {
  std::cout<<"sizeA  " <<configuration_.getCameras().size()<<"sizeB  " <<configuration_.getCamerasGt().size()<<std::endl;
  if (configuration_.getCameras().size() <= 3 || configuration_.getCamerasGt().size() <= 3) {
    std::cout << "GtComparator::registerCameras not enough cameras to estimate the rototranslation" << std::endl;
  }
  /*find rototrasl*/

  glm::vec3 accumulator = glm::vec3(0.0), accumulatorGt = glm::vec3(0.0);
  float count = 0;
  /*centroid cameras*/
  for (int curCam = 0;
      curCam
          < (configuration_.getCameras().size() < configuration_.getCamerasGt().size() ?
              configuration_.getCameras().size() : configuration_.getCamerasGt().size()); curCam++) {
    accumulator = accumulator + configuration_.getCameras()[curCam].center;
    accumulatorGt = accumulatorGt + configuration_.getCamerasGt()[curCam].center;
    count = count + 1.0;
  }
  std::vector<glm::vec3> newCenters, newCentersGt;

  glm::vec3 centroid = accumulator / count;
  glm::vec3 centroidGt = accumulatorGt / count;

  Eigen::Matrix4f M;
  M.setZero(4, 4);
  for (int curCam = 0;
      curCam
          < (configuration_.getCameras().size() < configuration_.getCamerasGt().size() ?
              configuration_.getCameras().size() : configuration_.getCamerasGt().size()); curCam++) {

    glm::vec3 curPt = configuration_.getCameras()[curCam].center - centroid;
    newCenters.push_back(curPt);
    glm::vec4 a = glm::vec4(0.0, curPt[0], curPt[1], curPt[2]);

    Eigen::Matrix4f Ma;
    Ma << a[0], -a[1], -a[2], -a[3],  //
    a[1], a[0], a[3], -a[2],  //
    a[2], -a[3], a[0], a[1], //
    a[3], a[2], -a[1], a[0];

    curPt = configuration_.getCamerasGt()[curCam].center - centroidGt;
    newCentersGt.push_back(curPt);
    glm::vec4 b = glm::vec4(0.0, curPt[0], curPt[1], curPt[2]);
    Eigen::Matrix4f Mb;
    Mb << b[0], -b[1], -b[2], -b[3],  //
    b[1], b[0], -b[3], b[2],  //
    b[2], b[3], b[0], -b[1], //
    b[3], -b[2], b[1], b[0];

    M = M + Ma.transpose() * Mb;

//    std::cout << "M: " << M << std::endl;
//    std::cout << "Ma: " << Ma << std::endl;
//    std::cout << "Mb: " << Mb << std::endl;

//    utilities::printMatrix(" configuration_.getCamerasGt()[curCam].center", configuration_.getCamerasGt()[curCam].center);
//    utilities::printMatrix("centroidGt",centroidGt);
//    utilities::printMatrix("curPt",curPt);
  }

  Eigen::Matrix4f ris;
  Eigen::EigenSolver<Eigen::Matrix4f> ges(M);

  Eigen::Matrix4f eig = ges.pseudoEigenvectors();

  int idx = 0;
  float maxeig = -1000000;
  for (int curR = 0; curR < 4; curR++) {
    if (ges.pseudoEigenvalueMatrix()(curR, curR) > maxeig) {
      maxeig = (float) ges.pseudoEigenvalueMatrix()(curR, curR);
      idx = curR;
    }
  }

  Eigen::Vector4f e = eig.col(idx);
  Eigen::Matrix4f M0;
  M0 << e[0], -e[1], -e[2], -e[3], //
  e[1], e[0], e[3], -e[2], //
  e[2], -e[3], e[0], e[1], //
  e[3], e[2], -e[1], e[0];
  Eigen::Matrix4f M1;
  M1 << e[0], -e[1], -e[2], -e[3], //
  e[1], e[0], -e[3], e[2], //
  e[2], e[3], e[0], -e[1], //
  e[3], -e[2], e[1], e[0];

  Eigen::Matrix4f R = M0.transpose() * M1;

//  std::cout << "M1: " << M1 << std::endl;
//  std::cout << "M0: " << M0 << std::endl;
//  std::cout << "M: " << M << std::endl;
//  std::cout << "eig: " << eig << std::endl;
//  std::cout << "vvv: " << ges.pseudoEigenvalueMatrix() << std::endl;
//  std::cout << "e: " << e << std::endl;

  Rsub = R.block<3, 3>(1, 1);

  float a = 0, b = 0;
  for (int curCam = 0; curCam < (newCenters.size()); curCam++) {

    glm::vec3 curPt = newCenters[curCam];
    glm::vec3 curPtGt = newCentersGt[curCam];
    Eigen::Vector3f va, vb;
    va << curPt[0], curPt[1], curPt[2];
    vb << curPtGt[0], curPtGt[1], curPtGt[2];

    a = a + (vb.transpose() * Rsub * va)[0];
    b = b + (vb.transpose() * vb)[0];
    //          a = a + Bn(:,i)'*R*An(:,i);
    //          b = b + Bn(:,i)'*Bn(:,i);
  }
  scale_ = b / a;

//  std::cout << "b: " << b << std::endl;
//  std::cout << "a: " << a << std::endl;

  Eigen::Vector3f centroidGtEig(centroidGt[0], centroidGt[1], centroidGt[2]);
  Eigen::Vector3f centroidEig(centroid[0], centroid[1], centroid[2]);

  //%Compute the final translation
  T = centroidGtEig - scale_ * Rsub * centroidEig;

//  std::cout << "centroidEig: " << centroidEig << std::endl;
//  std::cout << "centroidGtEig: " << centroidGtEig << std::endl;
  std::cout << "R: " << Rsub << std::endl;
  std::cout << "T: " << T << std::endl;
  std::cout << "s: " << scale_ << std::endl;

//  std::ofstream file1("centroid.txt");
//  std::ofstream file2("centroidGT.txt");
//  std::ofstream file3("centroidTransf.txt");
//  for (int curCam = 0;
//      curCam
//          < (configuration_.getCameras().size() < configuration_.getCamerasGt().size() ?
//              configuration_.getCameras().size() : configuration_.getCamerasGt().size()); curCam++) {
//    CameraType c = configuration_.getCameras()[curCam];
//    file1 << c.center[0] << " " << c.center[1] << " " << c.center[2] << " " << std::endl;
//  }
//  for (int curCam = 0;
//      curCam
//          < (configuration_.getCameras().size() < configuration_.getCamerasGt().size() ?
//              configuration_.getCameras().size() : configuration_.getCamerasGt().size()); curCam++) {
//    CameraType c = configuration_.getCamerasGt()[curCam];
//    file2 << c.center[0] << " " << c.center[1] << " " << c.center[2] << " " << std::endl;
//  }
//  for (int curCam = 0;
//      curCam
//          < (configuration_.getCameras().size() < configuration_.getCamerasGt().size() ?
//              configuration_.getCameras().size() : configuration_.getCamerasGt().size()); curCam++) {
//
//    CameraType c = configuration_.getCameras()[curCam];
//    Eigen::Vector3f va, f;
//    va << c.center[0], c.center[1], c.center[2];
//    f = scale_ * Rsub * va + T;
//
//    file3 << f[0] << " " << f[1] << " " << f[2] << " " << std::endl;
//  }
//  file1.close();
//  file2.close();
//  file3.close();
}

void GtComparator::printComparison() {

  std::cout << "Error statistics: " << std::endl;
  std::cout << "mean: " << res.mean << " stddev: " << res.stddev << std::endl;
  std::cout << "RMSE: " << res.rmse << " mae: " << res.mae << std::endl;

  float maxEl = *std::max_element(res.errs_.begin(), res.errs_.end());
  float minEl = *std::min_element(res.errs_.begin(), res.errs_.end());
  const float binSize = 0.02;
  int numBin = min(11, (int) ceil((maxEl - minEl) / binSize)); // requires <cmath>
  std::vector<int> histError(numBin, 0);

  for (auto e : res.errs_) {
    int bucket = (int) floor(fabs(e) / binSize);

    histError[bucket < numBin ? bucket : numBin - 1] += 1;
  }

  int totNum = std::accumulate(histError.begin(), histError.end(), 0);

  std::cout << "Errors histogram (%): " << std::endl;
  for (auto c : histError) {
    std::cout << setfill(' ') << setw(10) << std::fixed << std::setprecision(1);
    std::cout << 100.0 * static_cast<float>(c) / totNum << " ";
  }
  std::cout << std::endl;
  std::cout << "Errors histogram: (num)" << std::endl;
  for (auto c : histError) {
    std::cout << setfill(' ') << setw(10) << std::fixed << std::setprecision(1);
    std::cout << c << " ";
  }
  std::cout << std::endl;

}

void GtComparator::loadDepthMapGT(const std::string &pathGT_, cimg_library::CImg<float> & depth, int w, int h) {

  std::ifstream s;

  s.open(pathGT_.c_str(), std::ios::in);

  depth = cimg_library::CImg<float>(w, h);

  float mean = 0.0, tot = 0.0;
  for (int curR = 0; curR < h; ++curR) {
    std::string line;
    std::getline(s, line);
    std::istringstream iss(line);
    for (int curC = 0; curC < w; ++curC) {
      char c;
      iss >> depth(curC, curR);
      iss >> c;
    }
  }
}

} /* namespace reconstructorEvaluator */
