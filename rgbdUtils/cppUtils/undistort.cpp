#include <assert.h>
#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Utils.hpp"

using namespace std;
using namespace Utils;

void readCalib(const std::string& name, std::vector<std::string>& times) {
  std::ifstream fin(name.c_str());
  std::string line;
  while (std::getline(fin, line)) {
    times.push_back(line);
  }
}
void CreateFolder(const std::string& path) {
  const std::string path_make = "mkdir -p " + path;
  const int err = system(path_make.c_str());

  if (err == -1) {
    std::cout << "can't create " << path;
  }
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Example: ./projDepth calibFIle nameFile inDir outDir\n";
    return -1;
  }

  std::vector<std::string> hostNames;
  const std::string finCalib = argv[1];
  const std::string finNames = argv[2];
  std::string hostPath = argv[3], targetPath = argv[4];
  const int wTarget = 640, hTarget = 480;
  const double coeff = 1000.0;
  Utils::LoadSingleFile(finNames, hostNames);

  cv::FileStorage fs(finCalib, cv::FileStorage::READ);
  cv::Mat matK1, matDist1;
  fs["K1"] >> matK1;
  fs["dist1"] >> matDist1;
  fs.release();

  std::cout << "matK_h\n" << matK1 << "\n" << matK1.at<double>(0, 0) << "\n";
  std::cout << "matDist_t\n"
            << matDist1 << "\n"
            << matDist1.at<double>(0, 0) << "\n";

  if (hostPath.at(hostPath.length() - 1) != '/') hostPath = hostPath + "/";
  if (targetPath.at(targetPath.length() - 1) != '/')
    targetPath = targetPath + "/";
  CreateFolder(targetPath);

  for (const auto name : hostNames) {
    const std::string hostImgName = hostPath + name;
    const std::string targetImgName = targetPath + name;
    std::cout << "hostImgName " << hostImgName << std::endl;
    cv::Mat rawImg = cv::imread(hostImgName, CV_LOAD_IMAGE_COLOR);
    cv::Mat undistortImg;
    cv::undistort(rawImg, undistortImg, matK1, matDist1);

    cv::imwrite(targetImgName, undistortImg);
  }

  return 0;
}
