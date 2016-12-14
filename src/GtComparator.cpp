/*
 * GtComparator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#include <GtComparator.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/Polyhedron_scan_OFF.h>

namespace reconstructorEvaluator {


typedef CGAL::Simple_cartesian<double>               Kernel;
typedef Kernel::Point_3                              Point_3;
typedef CGAL::Polyhedron_3<Kernel>                   Polyhedron;
GtComparator::GtComparator(const std::string& path) {
  configuration_.setConfiguration(path);
}

GtComparator::~GtComparator() {
}

void GtComparator::run() {



}

} /* namespace reconstructorEvaluator */
