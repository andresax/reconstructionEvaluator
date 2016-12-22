/*
 * ParserInterface.hpp
 *
 *  Created on: Dec 22, 2016
 *      Author: andrea
 */

#ifndef INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_PARSERINTERFACE_HPP_
#define INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_PARSERINTERFACE_HPP_

#include <boost/filesystem.hpp>
#include <types.hpp>
namespace reconstructorEvaluator {
class ParserInterface {
public:
  virtual ~ParserInterface(){};
  virtual void parse(const boost::filesystem::path &path) = 0;

protected:
  std::vector<CameraType> cameras_;
};
}

#endif /* INCLUDE_RECONSTRUCTIONEVALUATOR_CAMPARSER_PARSERINTERFACE_HPP_ */
