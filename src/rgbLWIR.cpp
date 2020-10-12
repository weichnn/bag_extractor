/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <locale.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <tuple>

#include "cv_bridge/cv_bridge.h"
#include "dso_ros/Flag.h"
#include "geometry_msgs/Pose.h"
#include "math.h"

#define M_PI 3.14159265358979323846

std::string out_root = "./data";
//	left grey, left rgb, right optris, kinect rgb, kinect depth
std::tuple<std::string, std::string, std::string, std::string, std::string>
    outPaths;

bool useSampleOutput = false;
int flag_state_now = 0;
double offsetOptrisRGB = 0.0;

image_transport::Publisher mImg_pub;

void parseArgument(char *arg) {
  int option;
  char buf[1000];

  if (1 == sscanf(arg, "sampleoutput=%d", &option)) {
    if (option == 1) {
      useSampleOutput = true;
      printf("USING SAMPLE OUTPUT WRAPPER!\n");
    }
    return;
  }

  if (1 == sscanf(arg, "out_root=%s", buf)) {
    out_root = buf;
    printf("output path %s!\n", out_root.c_str());
    return;
  } else {
    printf("no out_root path !!\n");
    exit(1);
  }

  printf("could not parse argument \"%s\"!!\n", arg);
}

bool IsFileExist(const std::string &path) {
  if (access(path.c_str(), F_OK) == 0) {
    return true;
  } else {
    return false;
  }
}
void CreateFolder(const std::string &path) {
  //   if (IsFileExist(path)) {
  //     std::cout << path << "is already exist";
  //     return;
  //   }
  const std::string path_make = "mkdir -p " + path;
  const int err = system(path_make.c_str());
  //   const int err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH |
  //   S_IXOTH);
  if (err == -1) {
    std::cout << "can't create " << path;
  }
}

int frameID = 0;

void monoVidCb(const sensor_msgs::ImageConstPtr img) {
  sensor_msgs::Image copiedImg = *img;
  copiedImg.header.stamp = img->header.stamp + ros::Duration(offsetOptrisRGB);
  mImg_pub.publish(copiedImg);
}

void flagCb(const dso_ros::Flag flag) {
  flag_state_now = flag.flag_state;
  printf("%d\n", flag.flag_state);
}

void llh2xyz(double llh[3], double xyz[3])  //Î³¾­¸ß ×ª µØÐÄµØÇò×ø±êÏµ
{
  double phi = llh[0] * M_PI / 180.0f;
  double lambda = llh[1] * M_PI / 180.0f;
  double h = llh[2];

  double a = 6378137.0000f;  // earth semimajor axis in meters
  double b = 6356752.3142f;  // earth semiminor axis in meters
  double e = sqrt(1.0f - (b / a) * (b / a));

  double sinphi = sin(phi);
  double cosphi = cos(phi);
  double coslam = cos(lambda);
  double sinlam = sin(lambda);
  double tan2phi = (tan(phi)) * (tan(phi));
  double tmp = 1.0f - e * e;
  double tmpden = sqrt(1.0f + tmp * tan2phi);

  double x = (a * coslam) / tmpden + h * coslam * cosphi;

  double y = (a * sinlam) / tmpden + h * sinlam * cosphi;

  double tmp2 = sqrt(1.0f - e * e * sinphi * sinphi);
  double z = (a * tmp * sinphi) / tmp2 + h * sinphi;

  xyz[0] = x;
  xyz[1] = y;
  xyz[2] = z;
}

void xyz2enu(double xyz[3], double orgllh[3],
             double enu[3])  //µØÐÄµØÇò×ø±êÏµ ×ª ¶«±±ÌìµØÀí×ø±êÏµ
{
  double tmpxyz[3];
  double tmporg[3];
  double difxyz[3];
  // double orgllh[3];
  double orgxyz[3];
  double phi, lam, sinphi, cosphi, sinlam, coslam;

  llh2xyz(orgllh, orgxyz);

  int i;
  for (i = 0; i < 3; i++) {
    tmpxyz[i] = xyz[i];
    tmporg[i] = orgxyz[i];
    difxyz[i] = tmpxyz[i] - tmporg[i];
  }

  // xyz2llh(orgxyz,orgllh);

  phi = orgllh[0] * M_PI / 180.0f;
  lam = orgllh[1] * M_PI / 180.0f;
  sinphi = sin(phi);
  cosphi = cos(phi);
  sinlam = sin(lam);
  coslam = cos(lam);
  double R[3][3] = {{-sinlam, coslam, 0.0f},
                    {-sinphi * coslam, -sinphi * sinlam, cosphi},
                    {cosphi * coslam, cosphi * sinlam, sinphi}};

  enu[0] = 0;
  enu[1] = 0;
  enu[2] = 0;
  for (i = 0; i < 3; i++) {
    enu[0] = enu[0] + R[0][i] * difxyz[i];
    enu[1] = enu[1] + R[1][i] * difxyz[i];
    enu[2] = enu[2] + R[2][i] * difxyz[i];
  }
}

void gtCb(const sensor_msgs::NavSatFix nvdata) {
  FILE *f;
  char buf3[1000];
  snprintf(buf3, 1000, "%s/gt.txt", out_root.c_str());
  f = fopen(buf3, "a");

  double orgllh[3] = {(30.263254f), (120.115654f), 33.196903};
  double llh[3] = {(nvdata.latitude), (nvdata.longitude), nvdata.altitude};
  double xyz[3];
  llh2xyz(llh, xyz);
  double enu[3];
  llh2xyz(llh, xyz);
  xyz2enu(xyz, orgllh, enu);
  fprintf(f, "%lf %lf %lf %lf\n", nvdata.header.stamp.toSec(), enu[0], enu[1],
          enu[2]);
  fclose(f);
}

void vidCb(const sensor_msgs::ImageConstPtr img,
           const sensor_msgs::ImageConstPtr imgRight) {
  //	color
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  assert(cv_ptr->image.type() == CV_8UC3);
  assert(cv_ptr->image.channels() == 3);

  cv_bridge::CvImagePtr right_cv_ptr =
      cv_bridge::toCvCopy(imgRight, sensor_msgs::image_encodings::MONO16);
  assert(right_cv_ptr->image.type() == CV_16U);
  assert(right_cv_ptr->image.channels() == 1);

  cv::Mat m;
  cv::Mat v;
  cv::meanStdDev(right_cv_ptr->image, m, v);
  double maxinm;
  double mininm;
  cv::minMaxIdx(right_cv_ptr->image, &maxinm, &mininm);
  std::cout << m << "\t" << v << "\t" << maxinm << "\t" << mininm << std::endl;

  // cv::Mat min = m-3.0f*v;
  // cv::Mat max = m+3.0f*v;

  double min = mininm;
  double max = maxinm;

  double alpha = (255.0f) / (maxinm - mininm);

  for (int i = 0; i < (right_cv_ptr->image).rows; ++i) {
    for (int j = 0; j < (right_cv_ptr->image).cols; ++j) {
      double x = (double)(right_cv_ptr->image.at<ushort>(i, j)) - min;
      if (x < 0.0f) {
        right_cv_ptr->image.at<ushort>(i, j) = 0;
      } else {
        if (x > max) {
          right_cv_ptr->image.at<ushort>(i, j) = 255;
        } else {
          right_cv_ptr->image.at<ushort>(i, j) = alpha * x;
          // printf("%d\n", right_cv_ptr->image.at<ushort>(i,j));
        }
      }
    }
  }
  double time = cv_ptr->header.stamp.toSec();
  double right_time = right_cv_ptr->header.stamp.toSec();
  right_cv_ptr->image.convertTo(right_cv_ptr->image, CV_8U);

  printf("%f\n", time - right_time);

  //	use the same timestamps
  char bufgrey[1000];
  snprintf(bufgrey, 1000, "%s%lf.png", std::get<0>(outPaths).c_str(),
           cv_ptr->header.stamp.toSec());
  cv::Mat tis_grey;
  cv::cvtColor(cv_ptr->image, tis_grey, CV_BGR2GRAY);
  imwrite(bufgrey, tis_grey);

  char bufrgb[1000];
  snprintf(bufrgb, 1000, "%s/%lf.png", std::get<1>(outPaths).c_str(),
           cv_ptr->header.stamp.toSec());
  imwrite(bufrgb, cv_ptr->image);

  char bufoptris[1000];
  snprintf(bufoptris, 1000, "%s/%lf.png", std::get<2>(outPaths).c_str(),
           cv_ptr->header.stamp.toSec());
  imwrite(bufoptris, right_cv_ptr->image);

  FILE *f;
  char buf3[1000];
  snprintf(buf3, 1000, "%s/times.txt", out_root.c_str());
  f = fopen(buf3, "a");
  fprintf(f, "%lf\n", cv_ptr->header.stamp.toSec());
  fclose(f);

  FILE *fab;
  char buf4[1000];
  snprintf(buf4, 1000, "%s/ab.txt", out_root.c_str());
  fab = fopen(buf4, "a");
  fprintf(fab, "%lf %lf %lf\n", cv_ptr->header.stamp.toSec(), 1.0f / alpha,
          min);
  fclose(fab);

  FILE *fflag;
  char buf5[1000];
  snprintf(buf5, 1000, "%s/flag.txt", out_root.c_str());
  fflag = fopen(buf5, "a");
  fprintf(fflag, "%lf %d\n", cv_ptr->header.stamp.toSec(), flag_state_now);
  fclose(fflag);
}

void chatterCallback(const geometry_msgs::PoseStamped &msg) {
  double time = msg.header.stamp.toSec();

  FILE *fab;
  char buf4[1000];
  snprintf(buf4, 1000, "%s/groundtruth.txt", out_root.c_str());
  fab = fopen(buf4, "a");

  fprintf(fab, "%lf %lf %lf %lf %lf %lf %lf %lf\n", time, msg.pose.position.x,
          msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x,
          msg.pose.orientation.y, msg.pose.orientation.z,
          msg.pose.orientation.w);
  fclose(fab);
}

void chatterCallback2(const geometry_msgs::PoseStamped &msg) {
  double time = msg.header.stamp.toSec();

  FILE *fab;
  char buf4[1000];
  snprintf(buf4, 1000, "%s/groundtruth2.txt", out_root.c_str());
  fab = fopen(buf4, "a");

  fprintf(fab, "%lf %lf %lf %lf %lf %lf %lf %lf\n", time, msg.pose.position.x,
          msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x,
          msg.pose.orientation.y, msg.pose.orientation.z,
          msg.pose.orientation.w);
  fclose(fab);
}

void chatterCallback3(const geometry_msgs::PoseStamped &msg) {
  double time = msg.header.stamp.toSec();

  FILE *fab;
  char buf4[1000];
  snprintf(buf4, 1000, "%s/groundtruth3.txt", out_root.c_str());
  fab = fopen(buf4, "a");

  fprintf(fab, "%lf %lf %lf %lf %lf %lf %lf %lf\n", time, msg.pose.position.x,
          msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x,
          msg.pose.orientation.y, msg.pose.orientation.z,
          msg.pose.orientation.w);
  fclose(fab);
}

void imuCb(const sensor_msgs::Imu &msg) {
  double time = msg.header.stamp.toSec();

  FILE *fab;
  char buf[1000];
  snprintf(buf, 1000, "%s/imu.txt", out_root.c_str());
  fab = fopen(buf, "a");

  fprintf(fab, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n", time, msg.angular_velocity.x,
          msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x,
          msg.linear_acceleration.y, msg.linear_acceleration.z);
  fclose(fab);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "dso_live");

  for (int i = 1; i < argc; i++) parseArgument(argv[i]);

  if (out_root.at(out_root.length() - 1) != '/') out_root = out_root + "/";

  outPaths = std::make_tuple(out_root + "left/", out_root + "leftRGB/",
                             out_root + "right/", out_root + "kinectRGB/",
                             out_root + "kinectDepth/");
  std::cout << "save in " << out_root << std::endl;
  std::cout << "save rgb in " << std::get<0>(outPaths) << std::endl;
  CreateFolder(std::get<0>(outPaths));
  CreateFolder(std::get<1>(outPaths));
  CreateFolder(std::get<2>(outPaths));
  CreateFolder(std::get<3>(outPaths));
  CreateFolder(std::get<4>(outPaths));

  ros::NodeHandle nh("~");
  std::string imagetopic_L, imagetopic_R, imagetopic_kinectRGB,
      imagetopic_kinectD, imutopic;
  std::string poseTopic1, poseTopic2, poseTopic3;
  nh.getParam("imagetopic_l", imagetopic_L);
  nh.getParam("imagetopic_r", imagetopic_R);
  nh.getParam("imagetopic_krgb", imagetopic_kinectRGB);
  nh.getParam("imagetopic_d", imagetopic_kinectD);
  nh.getParam("offset", offsetOptrisRGB);
  nh.getParam("imuTopic", imutopic);
  nh.getParam("poseTopic1", poseTopic1);
  nh.getParam("poseTopic2", poseTopic2);
  std::cout << "offset between tis and optris: " << offsetOptrisRGB
            << std::endl;

  ros::Subscriber flagStateSub = nh.subscribe("/optris/flag_state", 1, &flagCb);
  ros::Subscriber gtSub = nh.subscribe("/fix", 1, &gtCb);
  ros::Subscriber imgSub = nh.subscribe(imagetopic_L, 1, &monoVidCb);
  ros::Subscriber imuSub = nh.subscribe(imutopic, 1, &imuCb);
  image_transport::ImageTransport it(nh);
  mImg_pub = it.advertise("/camera/image_raw2", 1);
  ros::Subscriber poseSub = nh.subscribe(poseTopic1, 10, &chatterCallback);
  ros::Subscriber poseSub2 = nh.subscribe(poseTopic2, 10, &chatterCallback2);

  message_filters::Subscriber<sensor_msgs::Image> left_sub(
      nh, "/camera/image_raw2", 10);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, imagetopic_R,
                                                            10);
  message_filters::Subscriber<sensor_msgs::Image> kinect_color_sub(
      nh, imagetopic_kinectRGB, 10);
  message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub(
      nh, imagetopic_kinectD, 10);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;

  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,
                                               right_sub);
  sync.registerCallback(boost::bind(&vidCb, _1, _2));

  ros::spin();

  return 0;
}
