#include "ClientNode.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "client node");

  ros::start();
  if(argc > 2) {
    ROS_WARN ("Arguments supplied via command line are neglected.");
  }

  ros::NodeHandle Nh;
  ros::NodeHandle NhPrivate("~");

  ClientNode cn (std::string(argv[1]), Nh, NhPrivate);

  /*string node_name = ros::this_node::getName();
  int Rate;
  NhPrivate.param(node_name+"/Rate",Rate,100);

  ros::Rate r(Rate);
  while(ros::ok())
  {
      ros::spinOnce();
      r.sleep();
  }*/

  ros::spin();

  ros::shutdown();

  return 0;
}

ClientNode::ClientNode(string sensor, ros::NodeHandle Nh, ros::NodeHandle NhPrivate){
  name_of_node_ = ros::this_node::getName();
  if(sensor=="MONO")
    sensor_ = ORB_SLAM3::MONOCULAR;
  else if(sensor=="STEREO")
    sensor_ = ORB_SLAM3::STEREO;
  else if(sensor=="RGBD")
    sensor_ = ORB_SLAM3::RGBD;
  else
    ROS_WARN ("The types are: MONO, STEREO, RGBD");
  nh_ = Nh;
  nhPrivate_ = NhPrivate;
  last_timestamp = 0;

  ORB_SLAM3::ORBParameters parameters;
  LoadOrbParameters (parameters);
  nh_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  //Construct the client system
  client_system_ = new ORB_SLAM3::ClientSystem(nh_, nhPrivate_, voc_file_name_param_, parameters, sensor_);

  if(sensor_ == ORB_SLAM3::MONOCULAR){
    image_subscriber = nhPrivate_.subscribe("/camera/image_raw", 10, &ClientNode::ImageCallbackMono, this);
  }
  else if(sensor_ == ORB_SLAM3::STEREO){
    left_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nhPrivate_, "image_left/image_color_rect", 1);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nhPrivate_, "image_right/image_color_rect", 1);
    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&ClientNode::ImageCallbackStereo, this, _1, _2));
  }
  else if(sensor_ == ORB_SLAM3::RGBD){
    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (nhPrivate_, "/camera/rgb/image_raw", 1);
    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (nhPrivate_, "/camera/depth_registered/image_raw", 1);
    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
    sync_->registerCallback(boost::bind(&ClientNode::ImageCallbackRGBD, this, _1, _2));
  }
  else
    ROS_WARN("Yo");
}
ClientNode::~ClientNode(){
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
  delete left_sub_;
  delete right_sub_;
}

void ClientNode::ImageCallbackRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  client_system_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

}

void ClientNode::ImageCallbackStereo (const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  client_system_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());

}

void ClientNode::ImageCallbackMono(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  if(cv_in_ptr->header.stamp.toSec()-last_timestamp>4 && cv_in_ptr->header.stamp.toSec()-last_timestamp<10000){
      std::cout<<"New Dataset"<<std::endl;
      client_system_->ChangeDataset();
  }
  client_system_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());
  last_timestamp = cv_in_ptr->header.stamp.toSec();
}

void ClientNode::LoadOrbParameters(ORB_SLAM3::ORBParameters& parameters){
    //ORB SLAM configuration parameters
  nh_.param(name_of_node_ + "/camera_fps", parameters.maxFrames, 30);
  nh_.param(name_of_node_ + "/camera_RGB", parameters.RGB, true);
  nh_.param(name_of_node_ + "/ORBextractor/nFeatures", parameters.nFeatures, 1200);
  nh_.param(name_of_node_ + "/ORBextractor/scaleFactor", parameters.scaleFactor, static_cast<float>(1.2));
  nh_.param(name_of_node_ + "/ORBextractor/nLevels", parameters.nLevels, 8);
  nh_.param(name_of_node_ + "/ORBextractor/iniThFAST", parameters.iniThFAST, 20);
  nh_.param(name_of_node_ + "/ORBextractor/minThFAST", parameters.minThFAST, 7);

/*  bool load_calibration_from_cam = false;
  nh_.param(name_of_node_ + "/load_calibration_from_cam", load_calibration_from_cam, false);*/

  //if (sensor_== ORB_SLAM3::System::STEREO || sensor_==ORB_SLAM3::System::RGBD) {
    nh_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
    nh_.param(name_of_node_ + "/DepthMapFactor", parameters.depthMapFactor, static_cast<float>(1.0));
  //}

  bool got_cam_calibration = true;
 /* if (sensor_== ORB_SLAM3::STEREO || sensor_==ORB_SLAM3::RGBD) {
    got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_baseline", parameters.baseline);
}*/ //TODO

  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_p1", parameters.p1);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_p2", parameters.p2);
  got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k3", parameters.k3);

  if (!got_cam_calibration) {
    ROS_ERROR ("Failed to get camera calibration parameters from the launch file.");
    throw std::runtime_error("No cam calibration");
  }
}
