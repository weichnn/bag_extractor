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
  //  host-kinect,2 targe-rgb,1
  cv::Mat matK_t, matDist_t, matK_h, matDist_h, matR_t_to_h, matt_t_to_h;
  fs["K1"] >> matK_t;
  fs["dist1"] >> matDist_t;
  fs["K2"] >> matK_h;
  fs["dist2"] >> matDist_h;
  fs["R1to2"] >> matR_t_to_h;
  fs["t1to2"] >> matt_t_to_h;
  fs.release();

  //  from host to target
  //  Eigen default is colMajor
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> hostK(
      matK_h.ptr<double>(), matK_h.rows, matK_h.cols);
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> targetK(
      matK_t.ptr<double>(), matK_t.rows, matK_t.cols);
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> eigenR(
      matR_t_to_h.ptr<double>(), matR_t_to_h.rows, matR_t_to_h.cols);
  Eigen::Vector3d eigent;
  cv::cv2eigen(matt_t_to_h, eigent);

  //  T_t_h, host to targe, 2 to 1
  //  T_h_t, target to host, 1 to 2
  Eigen::Matrix4d T_h_t = Eigen::Matrix4d::Identity();
  T_h_t.block<3, 3>(0, 0) = eigenR;
  T_h_t.block<3, 1>(0, 3) = eigent;
  Eigen::Matrix4d T_t_h = T_h_t.inverse();

  Eigen::Matrix3d hostKinv = hostK.inverse();

  std::cout << "T_h_t\n" << T_h_t << "\n";
  std::cout << "T_t_h\n" << T_t_h << "\n";
  std::cout << "hostK\n" << hostK << "\n" << hostK(0, 0) << "\n";
  std::cout << "matDist_h\n"
            << matDist_h << "\n"
            << matDist_h.at<double>(0, 0) << "\n";
  std::cout << "matK_h\n" << matK_h << "\n" << matK_h.at<double>(0, 0) << "\n";
  std::cout << "targetK\n" << targetK << "\n" << targetK(0, 0) << "\n";
  std::cout << "matDist_t\n"
            << matDist_t << "\n"
            << matDist_t.at<double>(0, 0) << "\n";
  // return -1;

  if (hostPath.at(hostPath.length() - 1) != '/') hostPath = hostPath + "/";
  if (targetPath.at(targetPath.length() - 1) != '/')
    targetPath = targetPath + "/";
  CreateFolder(targetPath);

  for (const auto name : hostNames) {
    const std::string hostImgName = hostPath + name;
    const std::string targetImgName = targetPath + name;
    std::cout << "hostImgName " << hostImgName << std::endl;
    cv::Mat hostDepth = cv::imread(hostImgName, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat undirstHostDepth;
    cv::undistort(hostDepth, undirstHostDepth, matK_h, matDist_h);
    undirstHostDepth.copyTo(hostDepth);

    const int w = hostDepth.cols;
    const int h = hostDepth.rows;
    cv::Mat targetDepth = cv::Mat::zeros(hTarget, wTarget, CV_16UC1);
    for (int i = 0; i < h; i++) {
      for (int j = 0; j < w; j++) {
        if (hostDepth.at<ushort>(i, j) <= 0) {
          continue;
        }
        const double depth =
            static_cast<double>(hostDepth.at<ushort>(i, j)) / coeff;
        assert(depth > 0 && depth <= 256);
        const Eigen::Vector3d pixel(static_cast<double>(j),
                                    static_cast<double>(i), 1.0);
        const Eigen::Vector3d pcHost = hostKinv * (pixel * depth);
        Eigen::Vector3d pcTarget =
            T_t_h.block<3, 3>(0, 0) * pcHost + T_t_h.block<3, 1>(0, 3);
        const double depthTarget = pcTarget(2);
        if (depthTarget <= 0.001) {
          continue;
        }
        pcTarget /= depthTarget;
        const Eigen::Vector3d pixelTarget = targetK * pcTarget;

        if ((pixelTarget(0) > 0) && (pixelTarget(0) < wTarget) &&
            (pixelTarget(1) > 0) && (pixelTarget(1) < hTarget)) {
          targetDepth.at<ushort>(pixelTarget(1), pixelTarget(0)) =
              static_cast<ushort>(depthTarget * coeff);
        }
      }
    }
    cv::imwrite(targetImgName, targetDepth);
  }

  return 0;
}
