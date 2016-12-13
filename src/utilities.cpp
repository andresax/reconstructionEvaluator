#include <utilities.hpp>

namespace utilities {

void readLineAndStore(std::ifstream &configFile, bool &value) {
  std::string line;
  int valueRead;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> valueRead;
  if (valueRead == 0) {
    value = false;
  } else {
    value = true;
  }
}

void readLineAndStore(std::ifstream &configFile, int &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}

void readLineAndStore(std::ifstream &configFile, double &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}
void readLineAndStore(std::ifstream &configFile, float &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}

void readLineAndStore(std::ifstream &configFile, std::string &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
  std::cout << iss.str() << std::endl;
  if (value.at(0) == '#') {
    value = std::string("");
  }
}

void readLineAndStore(std::ifstream &configFile, boost::filesystem::path &value) {
  std::string valueTMP;
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> valueTMP;
  std::cout << iss.str() << std::endl;
  if (valueTMP.at(0) == '#') {
    valueTMP = std::string("");
  }
  value = valueTMP;

}


void printMatrix(glm::mat4 matrix) {
  for (int curRow = 0; curRow < 4; ++curRow) {
    for (int curCol = 0; curCol < 4; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}
void printMatrix(const std::string message, glm::mat4 matrix) {
  std::cout << message << std::endl;
  for (int curRow = 0; curRow < 4; ++curRow) {
    for (int curCol = 0; curCol < 4; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}
void printMatrix(glm::mat3 matrix) {
  for (int curRow = 0; curRow < 3; ++curRow) {
    for (int curCol = 0; curCol < 3; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}

void printMatrix(std::string message, glm::mat3 matrix) {
  std::cout << message << std::endl;
  for (int curRow = 0; curRow < 3; ++curRow) {
    for (int curCol = 0; curCol < 3; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}

void printMatrix(glm::vec3 vector) {
  for (int curIdx = 0; curIdx < 3; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

void printMatrix(std::string message, glm::vec4 vector) {
  std::cout << message << std::endl;
  printMatrix(vector);
}
void printMatrix(glm::vec4 vector) {
  for (int curIdx = 0; curIdx < 4; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

void printMatrix(std::string message, glm::vec3 vector) {
  std::cout << message << std::endl;
  for (int curIdx = 0; curIdx < 3; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}


}

