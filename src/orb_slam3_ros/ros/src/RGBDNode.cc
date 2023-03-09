#include "RGBDNode.h"

void LoadOrbParameters(ros::NodeHandle nh_, ORB_SLAM3::ORBParameters& parameters, string name_of_node_, ORB_SLAM3::eSensor sensor_){
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

  
  nh_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
  nh_.param(name_of_node_ + "/DepthMapFactor", parameters.depthMapFactor, static_cast<float>(1.0));
  nh_.param(name_of_node_ + "/camera_bf", parameters.camera_bf, static_cast<float>(1.0));

  bool got_cam_calibration = true;

  nh_.param(name_of_node_ + "/Camera_type", parameters.camera_type, string("PinHole"));
  if(parameters.camera_type=="PinHole"){
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_p1", parameters.p1);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_p2", parameters.p2);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k3", parameters.k3);
  }
  else if(parameters.camera_type=="KannalaBrandt8"){
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k3", parameters.k3);
      got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_k4", parameters.k4);
  }


  if (!got_cam_calibration) {
    ROS_ERROR ("Failed to get camera calibration parameters from the launch file.");
    throw std::runtime_error("No cam calibration");
  }
  

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "client node");

  ros::start();
  if(argc > 2) {
    ROS_WARN ("Arguments supplied via command line are neglected.");
  }

  ros::NodeHandle Nh;
  ros::NodeHandle NhPrivate("~");

  string name_of_node_ = ros::this_node::getName();
  ORB_SLAM3::ORBParameters parameters;
  LoadOrbParameters(NhPrivate, parameters, name_of_node_, ORB_SLAM3::RGBD);
  string voc_file_name_param_;
  NhPrivate.param<string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  string file_path;
  NhPrivate.param<string>(name_of_node_ + "/save_path", file_path, "file_not_set");
  //Construct the client system
  ORB_SLAM3::ClientSystem client_system_(Nh, NhPrivate, voc_file_name_param_, parameters, ORB_SLAM3::RGBD);

  ImageGrabber igb(&client_system_,true);
 /* message_filters::Subscriber<sensor_msgs::Image> rgb_subscriber_(NhPrivate, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_subscriber_(NhPrivate, "/camera/depth_registered/image_raw", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_subscriber_, depth_subscriber_, 10);
  sync.registerCallback(boost::bind(&ImageGrabber::ImageCallbackRGBD, &igb, _1, _2));*/

  message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_, *depth_subscriber_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> *sync_;

  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (NhPrivate, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (NhPrivate, "/camera/depth_registered/image_raw", 1);
  
  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&ImageGrabber::ImageCallbackRGBD, &igb ,_1, _2));


  //std::thread sync_thread(&ImageGrabber::rgbdSync,&igb);

  ros::spin();

  ros::shutdown();
  //client_system_.SaveKeyFrameTrajectoryEuRoC(file_path);

  return 0;
}
void ImageGrabber::ImageCallbackRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD){
  mBufMutex.lock();
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  //std::cout<<"before cbRGBD"<<std::endl;
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
  /*if(!rgbdBuf.empty())
    rgbdBuf.pop();
  rgbdBuf.push(std::make_pair(msgRGB, msgD));
  //std::cout<<"after cbRGBD"<<std::endl;*/
  mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
  mBufMutex.unlock();
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  //std::cout<<"GrabImage"<<std::endl;
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg, const string typeName)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  if(typeName == "MONO8"){
    try
    {
      cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  else if(typeName == "TYPE_32FC1"){
    try
    {
      cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  return cv_ptr->image.clone();
}
void ImageGrabber::rgbdSync(){
  while(1)
  {
    cv::Mat imRGB;
    cv::Mat imD;
    double tIm = 0;
    
    if (!rgbdBuf.empty())
    {
      //std::cout << "0" << std::endl;
      tIm = rgbdBuf.front().first->header.stamp.toSec();
      {
        this->mBufMutex.lock();
        imRGB = GetImage(rgbdBuf.front().first, "MONO8");
        imD = GetImage(rgbdBuf.front().second, "TYPE_32FC1");
        rgbdBuf.pop();
        this->mBufMutex.unlock();
      }
      //std::cout << "1" << std::endl;
      mpSLAM->TrackRGBD(imRGB, imD, tIm);
    }
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}


ImageGrabber::ImageGrabber(ORB_SLAM3::ClientSystem* pSLAM, const bool bClahe):
        mpSLAM(pSLAM),  mbClahe(bClahe)
{
    dR_last = 0;
    tR_last = 0;
}