/*
 * DepthMapFromMesh.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: andrea
 */

#include <DepthMapFromMesh.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>

namespace reconstructorEvaluator {

typedef Polyhedron::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::FT FT;
typedef Kernel::Segment_3 Segment;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

DepthMapFromMesh::DepthMapFromMesh(Polyhedron *mesh) {
  mesh_ = mesh;

}

DepthMapFromMesh::~DepthMapFromMesh() {
}

void DepthMapFromMesh::computeMap(const CameraType& cam,int num) {
  curCam = cam;
  glm::vec3 curRay, curIntersection;

  Tree tree(faces(*mesh_).first, faces(*mesh_).second, *mesh_);

  depth = cimg_library::CImg<float>(cam.imageWidth, cam.imageHeight);
  depth.fill(-1.0);
//  std::ofstream file("outMesh3.ply");
//  std::ofstream file2("outMesh2.ply");
  std::vector<Ray> rays;
  for (int row = 0; row < cam.imageWidth; ++row) {
    for (int col = 0; col < cam.imageHeight; ++col) {
      computeRayFromCurCam(row, col, curRay);
//      file2<< curRay.x<<" "<<curRay.y<<" "<<curRay.z<<std::endl;
      Ray ray(Point(cam.center.x, cam.center.y, cam.center.z), Vector(curRay.x, curRay.y, curRay.z));
      rays.push_back(ray);
      //Segment seg(Point(cam.center.x, cam.center.y, cam.center.z), Point(cam.center.x, cam.center.y, cam.center.z)+2000.0*Vector(curRay.x, curRay.y, curRay.z));
      Ray_intersection intersection = tree.first_intersection(ray);
      if (intersection) {
        // gets intersection object
        const Point* p = boost::get<Point>(&(intersection->first));
        if (p ) {
          float distance = glm::length(cam.center - glm::vec3(p->x(), p->y(), p->z()));
          glm::vec3 dd= ( cam.center - glm::vec3(p->x(), p->y(), p->z()));
//          file<< 5.20606*dd.x<<" "<<5.20606*dd.y<<" "<<5.20606*dd.z<<std::endl;

          if (distance > 0.0 && (distance < depth(row, col) || depth(row, col) < 0.0)) {
            depth(row, col) = distance;
          }
          //std::cout << "intersection object is a point " << *p << std::endl;
        }

      }

    }
  }
//  file.close();
//  file2.close();
  //printRays(rays,num);

  depth.save_ascii("depthMesh.txt");

  cimg_library::CImg<float> deptht=depth;
  deptht.normalize(0, 255);
  deptht.save_png("depthMesh.png");

}

void DepthMapFromMesh::computeRayFromCurCam(const float & x, const float & y, glm::vec3 &ray) {

  glm::vec3 vecCenterpt2d = glm::normalize(
      glm::vec3((x - curCam.intrinsics[0][2]) / curCam.intrinsics[0][0], (y - curCam.intrinsics[1][2]) / curCam.intrinsics[1][1], 1.0));

  glm::vec4 vv = (curCam.extrinsics) * glm::vec4(vecCenterpt2d, 1.0);
//  utilities::printMatrix("curCam.extrinsics",curCam.extrinsics);
//  ray = glm::normalize(glm::vec3(vv[0] / vv[3], vv[1] / vv[3], vv[2] / vv[3]));
  ray = glm::normalize(glm::vec3(vv[0], vv[1], vv[2] ));
//utilities::printMatrix("vv", vv);
//  glm::vec3 check = ray * curCam.rotation   + curCam.translation;

  //ray = glm::vec3(vv.x,vv.y,vv.z);
}

void DepthMapFromMesh::printRays(const std::vector<Ray> &rays,int num) {

  std::stringstream s;
  s<<"rays/rays"<<num<<".ply";
  std::ofstream visFile(s.str());
//
  int numVisRays = rays.size();

  std::cout << "Writing initVis_" << std::endl;
  std::cout << "numVisRays_: " << numVisRays << std::endl;

  visFile << "ply" << std::endl << "format ascii 1.0" << std::endl;
  visFile << "element vertex " << numVisRays * 2 << std::endl;
  visFile << "property float x " << std::endl << "property float y " << std::endl << "property float z " << std::endl;
  visFile << "element edge " << numVisRays << std::endl;
  visFile << "property int vertex1 " << std::endl << "property int vertex2" << std::endl;
  visFile << "end_header" << std::endl;

  for (auto r : rays) {


    Point s = r.source();
    Point t = (r.source() + 2*r.to_vector());
    visFile << s.x() << " " << s.y() << " " << s.z() << std::endl;
    visFile << t.x() << " " << t.y() << " " << t.z() << std::endl;
  }

  for (int curPtIdx = 0; curPtIdx < rays.size()*2; curPtIdx += 2) {
    visFile << curPtIdx << " " << curPtIdx + 1 << std::endl;
  }
  visFile.close();
}

} /* namespace reconstructorEvaluator */
