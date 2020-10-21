/*
 * @Author: chen shenzhou
 * @Date: 2019-06-19 15:53:50
 * @Last Modified by: chen shenzhou
 * @Last Modified time: 2019-06-19 21:08:16
 */
#include "Utils.hpp"

namespace Utils {

void LoadSingleFile(const std::string &File, std::vector<std::string> &vNames) {
  std::ifstream fFile(File);
  if (!fFile.is_open()) {
    cerr << "open file " << File << "failed." << endl;
    return;
  }
  std::string line;
  while (getline(fFile, line)) {
    std::string name = line.substr(0, line.find("\n"));
    vNames.push_back(name);
  }
}

}  //  namespace Utils
