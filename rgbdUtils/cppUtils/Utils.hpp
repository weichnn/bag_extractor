/*
 * @Author: chen shenzhou
 * @Date: 2019-06-19 15:53:47
 * @Last Modified by: chen shenzhou
 * @Last Modified time: 2019-06-19 20:50:01
 */

/*! \mainpage PCLutils Library
 *
 * PCL utils library for C++.
 *
 * Written by shenzhou chen,
 * ZheJiang University
 *
 * \section requirements Requirements
 * This library requires the PCL, Eigen and OpenCV libraries.
 *
 */

#ifndef UTILS_H
#define UTILS_H

#include <assert.h>
#include <math.h>
#include <opencv2/videoio/videoio_c.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using std::cerr;
using std::cout;
using std::endl;
using std::string;

using namespace Eigen;
using namespace cv;

namespace Utils {

/**  @brief the function to read each line of string data in a txt file
 *
 * @param File path of file.
 * @param vNames vector of date
 */
void LoadSingleFile(const std::string &File, std::vector<std::string> &vNames);

}  //  namespace Utils
#endif  // UTILS_H
