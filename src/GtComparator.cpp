/*
 * GtComparator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <GtComparator.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <Polyhedron_AssimpToCgal.h>

namespace reconstructorEvaluator {

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
GtComparator::GtComparator(const std::string& path) {
  configuration_.setConfiguration(path);
}

GtComparator::~GtComparator() {

}

void GtComparator::run() {
  Polyhedron p;

  Builder_dae<HalfedgeDS> poly_builder(configuration_.getMeshPath());
  p.delegate(poly_builder);

  std::ofstream fileTest("testMesh.off");
  fileTest<<p;

}

} /* namespace reconstructorEvaluator */
