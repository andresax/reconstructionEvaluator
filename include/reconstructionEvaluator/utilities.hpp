#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <types.hpp>
#include <boost/filesystem.hpp>

namespace utilities {

/**
 * Read one line in the configuration file and stores the parameter in the value variable
 */
void readLineAndStore(std::ifstream &configFile, bool &value);
void readLineAndStore(std::ifstream &configFile, int &value);
void readLineAndStore(std::ifstream &configFile, double &value);
void readLineAndStore(std::ifstream &configFile, float &value);
void readLineAndStore(std::ifstream &configFile, std::string &value);
void readLineAndStore(std::ifstream &configFile, boost::filesystem::path &value);
std::string getFrameNumber(int curFrame, int digitIdxLength) ;
void printMatrix(glm::mat4 matrix);
void printMatrix(std::string message, glm::mat4 matrix);
void printMatrix(glm::mat3 matrix);
void printMatrix(std::string message, glm::mat3 matrix);
void printMatrix(glm::vec3 vector);
void printMatrix(std::string message, glm::vec3 vector);
void printMatrix(glm::vec4 vector);
void printMatrix(std::string message, glm::vec4 vector);

}  // namespace utils

#endif /* UTILITIES_HPP_ */
