
#include "callbacker.cpp"
using namespace msdi;

int main(int argc, char **argv) {
  ros::init(argc, argv, "msdi_ros_live");

  for (int i = 1; i < argc; i++) parseArgument(argv[i]);

  if (out_root.at(out_root.length() - 1) != '/') out_root = out_root + "/";

  constructFolder();

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
  ros::Subscriber imuSub = nh.subscribe(imutopic, 1, &imuCb);
  image_transport::ImageTransport it(nh);
  mImg_pub = it.advertise("/camera/image_raw2", 1);
  ros::Subscriber poseSub = nh.subscribe(poseTopic1, 10, &chatterCallback);
  ros::Subscriber poseSub2 = nh.subscribe(poseTopic2, 10, &chatterCallback2);

  ros::Subscriber kinect_color_raw_sub =
      nh.subscribe(imagetopic_kinectRGB, 3, &kinectRGBCb);
  ros::Subscriber kinect_depth_raw_sub =
      nh.subscribe(imagetopic_kinectD, 3, &kinectDepthCb);

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
