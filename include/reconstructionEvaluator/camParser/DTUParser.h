#ifndef SRC_CAMPARSER_DTUPARSER_H_
#define SRC_CAMPARSER_DTUPARSER_H_

#include <boost/filesystem.hpp>
#include <ParserInterface.hpp>
#include <types.hpp>
namespace reconstructorEvaluator {

class DTUParser : public ParserInterface {
public:
  DTUParser();
  virtual ~DTUParser();

  void parse(const boost::filesystem::path &path);

  const std::vector<CameraType>& getCameras() const {
    return cameras_;
  }
private:
  void orderPaths(const boost::filesystem::path &path,std::vector<boost::filesystem::path> &paths);
  std::vector<boost::filesystem::path> cameraPaths_;
};

} /* namespace reconstructorEvaluator */

#endif /* SRC_CAMPARSER_DTUPARSER_H_ */
