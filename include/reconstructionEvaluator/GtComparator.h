/*
 * GtComparator.h
 *
 *  Created on: Dec 13, 2016
 *      Author: andrea
 */

#ifndef SRC_GTCOMPARATOR_H_
#define SRC_GTCOMPARATOR_H_

#include <string>
#include <vector>
#include <types.hpp>
#include <Configuration.h>
#include <Configuration.h>
namespace reconstructorEvaluator {

class GtComparator {
public:
  GtComparator(const std::string &path);
  virtual ~GtComparator();
  void run();
private:
  Configuration configuration_;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_GTCOMPARATOR_H_ */
