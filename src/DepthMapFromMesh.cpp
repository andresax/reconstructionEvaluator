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
namespace reconstructorEvaluator {

typedef Polyhedron::Point_3 Point;
typedef Kernel::Ray_3 Ray;
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

void DepthMapFromMesh::computeMap(const CameraType& cam) {
  curCam = cam;
  glm::vec3 curRay, curIntersection;

  Tree tree(faces(*mesh_).first, faces(*mesh_).second, *mesh_);

  cimg_library::CImg<float> depth(cam.imageWidth, cam.imageHeight);
  depth.fill(-1.0);
  for (int row = 0; row < cam.imageWidth; ++row) {
    for (int col = 0; col < cam.imageHeight; ++col) {
      computeRayFromCurCam(row, col, curRay);
      Ray ray(Point(cam.center.x, cam.center.y, cam.center.z), Vector(curRay.x, curRay.y, curRay.z));
      //Segment seg(Point(cam.center.x, cam.center.y, cam.center.z), Point(cam.center.x, cam.center.y, cam.center.z)+2000.0*Vector(curRay.x, curRay.y, curRay.z));
      Ray_intersection intersection = tree.any_intersection(ray);
      if (intersection) {
        // gets intersection object
        const Point* p = boost::get<Point>(&(intersection->first));
        if (p) {
          float distance = glm::length(cam.center - glm::vec3(p->x(), p->y(), p->z()));


          if (distance > 0.0 && (distance < depth(row, col) || depth(row, col) < 0.0)) {
            depth(row, col) = distance;
          }
          //std::cout << "intersection object is a point " << *p << std::endl;
        }

      }

    }
  }
  depth.save_ascii("depthMesh.txt");

  depth.normalize(0,255);
  depth.save_png("depthMesh.png");

}

void DepthMapFromMesh::computeRayFromCurCam(const float & x, const float & y, glm::vec3 &ray) {

  glm::vec3 vecCenterpt2d = glm::normalize(
      glm::vec3((x - curCam.intrinsics[0][2]) / curCam.intrinsics[0][0], (y - curCam.intrinsics[1][2]) / curCam.intrinsics[1][1], 1.0));

  glm::vec4 vv = (curCam.extrinsics) * glm::vec4(vecCenterpt2d, 1.0);
  ray = glm::normalize(glm::vec3(vv[0] / vv[3], vv[1] / vv[3], vv[2] / vv[3]));
}

void DepthMapFromMesh::intersectRayMesh(const glm::vec3& ray, glm::vec3 &intersection) {

}

} /* namespace reconstructorEvaluator */
