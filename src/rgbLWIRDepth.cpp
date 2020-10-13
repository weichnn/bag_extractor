
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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <tuple>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "math.h"
#include "msdi_ros/Flag.h"

std::string out_root = "./data";
std::tuple<std::string, std::string, std::string, std::string, std::string>
    outPaths;

bool useSampleOutput = false;
int flag_state_now = 0;
double offsetOptrisRGB = 0.0;

image_transport::Publisher mImg_pub;

void parseArgument(char *arg) {
  int option;
  char buf[1000];

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
  const std::string path_make = "mkdir -p " + path;
  const int err = system(path_make.c_str());
  if (err == -1) {
    std::cout << "can't create " << path;
  }
}

cv::Mat img_rot180(const cv::Mat &img) {
  cv::Point2f src_center(img.cols / 2.0F, img.rows / 2.0F);
  cv::Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
  cv::Mat dst;
  cv::warpAffine(img, dst, rot_mat, img.size());

  return dst;
}

cv::Mat im_rotate_crop(const cv::Mat &img, const cv::Rect &rect) {
  cv::Point2f src_center(img.cols / 2.0F, img.rows / 2.0F);
  cv::Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
  cv::Mat dst;
  cv::warpAffine(img, dst, rot_mat, img.size());

  cv::Mat ROI(dst, rect);
  cv::Mat croppedImage;
  ROI.copyTo(croppedImage);

  return croppedImage;
}
cv::Mat im_crop(const cv::Mat &img, const cv::Rect &rect) {
  cv::Mat ROI(img, rect);
  cv::Mat croppedImage;
  ROI.copyTo(croppedImage);

  return croppedImage;
}

void monoVidCb(const sensor_msgs::ImageConstPtr img) {
  sensor_msgs::Image copiedImg = *img;
  copiedImg.header.stamp = img->header.stamp + ros::Duration(offsetOptrisRGB);
  mImg_pub.publish(copiedImg);
}

void flagCb(const msdi_ros::Flag flag) {
  flag_state_now = flag.flag_state;
  printf("%d\n", flag.flag_state);
}

//	without kinect rgb
void mssensorCb(const sensor_msgs::ImageConstPtr img,
                const sensor_msgs::ImageConstPtr imgRight,
                const sensor_msgs::ImageConstPtr imgKinectDepth) {
  //	color
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  assert(cv_ptr->image.type() == CV_8UC3);
  assert(cv_ptr->image.channels() == 3);

  //	optris
  cv_bridge::CvImagePtr right_cv_ptr =
      cv_bridge::toCvCopy(imgRight, sensor_msgs::image_encodings::MONO16);
  assert(right_cv_ptr->image.type() == CV_16U);
  assert(right_cv_ptr->image.channels() == 1);

  //	kinect
  cv::Rect rect(160, 30, 640, 480);

  cv_bridge::CvImagePtr kinect_depth_cv_ptr = cv_bridge::toCvCopy(
      imgKinectDepth, sensor_msgs::image_encodings::TYPE_16UC1);
  assert(kinect_depth_cv_ptr->image.type() == CV_16UC1);
  assert(kinect_depth_cv_ptr->image.channels() == 1);
  // kinect_depth_cv_ptr->image = im_rotate_crop(kinect_depth_cv_ptr->image,
  // rect);
  kinect_depth_cv_ptr->image = img_rot180(kinect_depth_cv_ptr->image);

  //	process optris
  cv::Mat m;
  cv::Mat v;
  cv::meanStdDev(right_cv_ptr->image, m, v);
  double maxinm;
  double mininm;
  cv::minMaxIdx(right_cv_ptr->image, &maxinm, &mininm);
  std::cout << "optris mean, std, max, min" << m << "\t" << v << "\t" << maxinm
            << "\t" << mininm << std::endl;

  // cv::Mat min = m - 3.0f * v;
  // cv::Mat max = m + 3.0f * v;

  double min = mininm;
  double max = maxinm;

  double alpha = (255.0f) / (maxinm - mininm);

  for (int i = 0; i < (right_cv_ptr->image).rows; ++i) {
    for (int j = 0; j < (right_cv_ptr->image).cols; ++j) {
      double x =
          (double)(right_cv_ptr->image.at<ushort>(i, j)) - min;
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

  char bufkd[1000];
  snprintf(bufkd, 1000, "%s/%lf.png", std::get<4>(outPaths).c_str(),
           cv_ptr->header.stamp.toSec());
  imwrite(bufkd, kinect_depth_cv_ptr->image);

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

  fprintf(fab, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n", time,
          msg.angular_velocity.x, msg.angular_velocity.y,
          msg.angular_velocity.z, msg.linear_acceleration.x,
          msg.linear_acceleration.y, msg.linear_acceleration.z);
  fclose(fab);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "msdi_ros_live");

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
  ros::Subscriber imgSub = nh.subscribe(imagetopic_L, 1, &monoVidCb);
  image_transport::ImageTransport it(nh);
  mImg_pub = it.advertise("/camera/image_raw2", 1);
  ros::Subscriber imuSub = nh.subscribe(imutopic, 1, &imuCb);
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

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
      sync_pol3;
  message_filters::Synchronizer<sync_pol3> sync3(sync_pol3(10), left_sub,
                                                 right_sub, kinect_depth_sub);
  sync3.registerCallback(boost::bind(&mssensorCb, _1, _2, _3));

  ros::spin();

  return 0;
}
