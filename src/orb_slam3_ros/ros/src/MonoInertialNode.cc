#include "MonoInertialNode.h"

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

  if (sensor_!= ORB_SLAM3::IMU_MONOCULAR && sensor_==ORB_SLAM3::MONOCULAR) {
    nh_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
    nh_.param(name_of_node_ + "/DepthMapFactor", parameters.depthMapFactor, static_cast<float>(1.0));
  }

  bool got_cam_calibration = true;
 if (sensor_== ORB_SLAM3::STEREO || sensor_==ORB_SLAM3::RGBD) {
    got_cam_calibration &= nh_.getParam(name_of_node_ + "/camera_baseline", parameters.baseline);
  } //TODO

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
  if (sensor_== ORB_SLAM3::IMU_MONOCULAR){
    bool got_imu_calibration = true;
    cv::Mat Tbc(4,4, CV_32F);
    XmlRpc::XmlRpcValue TbcConfig;
    if(!nh_.getParam(name_of_node_ + "/Tbc_data", TbcConfig))
        ROS_ERROR("Failed to get parameter from server.");
  //  ROS_ASSERT(TbcConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
    int matSize = 4;
    for (int i = 0; i < matSize; i++)
    {
      for (int j = 0; j < matSize; j++)
      {
          XmlRpc::XmlRpcValue tmp_value = TbcConfig[matSize * i + j];
          //std::cout<<double(tmp_value)<<std::endl;
          if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
              Tbc.at<float>(i, j) = double(tmp_value);
      }
    }
    parameters.Tbc = Tbc;

    got_imu_calibration &= nh_.getParam(name_of_node_ + "/IMU_Frequency", parameters.freq);
    got_imu_calibration &= nh_.getParam(name_of_node_ + "/IMU_NoiseGyro", parameters.Ng);
    got_imu_calibration &= nh_.getParam(name_of_node_ + "/IMU_NoiseAcc", parameters.Na);
    got_imu_calibration &= nh_.getParam(name_of_node_ + "/IMU_GyroWalk", parameters.Ngw);
    got_imu_calibration &= nh_.getParam(name_of_node_ + "/IMU_AccWalk", parameters.Naw);
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
  LoadOrbParameters(NhPrivate, parameters, name_of_node_, ORB_SLAM3::IMU_MONOCULAR);
  string voc_file_name_param_;
  NhPrivate.param<string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  string file_path;
  NhPrivate.param<string>(name_of_node_ + "/save_path", file_path, "file_not_set");
  //Construct the client system
  ORB_SLAM3::ClientSystem client_system_(Nh, NhPrivate, voc_file_name_param_, parameters, ORB_SLAM3::IMU_MONOCULAR);

  ImuGrabber imugb;
  GTGrabber gtgb;
  ImageGrabber igb(&client_system_,&imugb,&gtgb,true); // TODO

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = NhPrivate.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  ros::Subscriber sub_gt = NhPrivate.subscribe("/gt_pos", 100, &GTGrabber::GrabGT, &gtgb);
  ros::Subscriber sub_img0 = NhPrivate.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
  //std::thread sync_thread(&ImageGrabber::SyncWithImuAndGT,&igb);

  ros::spin();

  ros::shutdown();
  client_system_.SaveKeyFrameTrajectoryEuRoC(file_path);

  return 0;
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

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    //std::cout<<"1"<<std::endl;
    //std::cout<<"img0Buf:"<<img0Buf.size()<<std::endl;
    //std::cout<<"imuBuf:"<<mpImuGb->imuBuf.size()<<std::endl;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      //std::cout<<"2"<<std::endl;
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          //std::cout<<"acc:"<<acc<<std::endl;
        //  std::cout<<"gyr:"<<gyr<<std::endl;
        //  std::cout<<"t:"<<t<<std::endl;
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

    //  std::cout<<"before TrackMonocular"<<std::endl;
      mpSLAM->TrackMonocular(im,tIm,vImuMeas);
  //    std::cout<<"after TrackMonocular"<<std::endl;
    }
    //std::cout<<"3"<<std::endl;
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
    //std::cout<<"4"<<std::endl;
  }
}
void ImageGrabber::SyncWithImuAndGT()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
    /*  if(tIm>mpGTGb->GTBuf.back()->header.stamp.toSec())
          continue;*/

      //IMAGE
      {
          this->mBufMutex.lock();
          im = GetImage(img0Buf.front());
          img0Buf.pop();
          this->mBufMutex.unlock();
      }
      //IMU
      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();

      //GT
      double tR=0;
      float dR=0;
      mpGTGb->mBufMutex.lock();
      if(!mpGTGb->GTBuf.empty())
      {
        // Load measurements from buffer
        while(!mpGTGb->GTBuf.empty() && mpGTGb->GTBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpGTGb->GTBuf.front()->header.stamp.toSec();
          float x, y, z;
          x = mpGTGb->GTBuf.front()->point.x;
          y = mpGTGb->GTBuf.front()->point.y;
          z = mpGTGb->GTBuf.front()->point.z;
          float d = sqrt(x*x+y*y+z*z);
          mpGTGb->GTBuf.pop();
          tR=t;
          dR=d;
        }
      }
      mpGTGb->mBufMutex.unlock();
      if(tR==0){
         // std::cout<<"GT?"<<std::endl;
          //continue;
          dR = dR_last;
          tR = tR_last;
      }
      if(tR==0){
          //std::cout<<"GT_last 0"<<std::endl;
          continue;
      }
      double deltaT = tIm-tR;
      ORB_SLAM3::Range* Range_GT = new ORB_SLAM3::Range(dR,tR,deltaT);
      dR_last = dR;
      tR_last = tR;

      if(mbClahe)
        mClahe->apply(im,im);

      //std::cout<<"Range_GT:"<<Range_GT->d<<std::endl;
      //std::cout<<"tIm:"<<tIm<<",tR:"<<tR<<std::endl;
     // std::cout<<"delta t:"<<Range_GT->delta_t<<std::endl;
      mpSLAM->TrackMonocular(im,tIm,vImuMeas,Range_GT);
    }
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}
ImuGrabber::ImuGrabber(){
    //std::cout<<"Construct imuBuf:"<<imuBuf.size()<<std::endl;
}
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
//  std::cout<<"GrabImu"<<std::endl;
  mBufMutex.unlock();
  return;
}
ImageGrabber::ImageGrabber(ORB_SLAM3::ClientSystem* pSLAM, ImuGrabber *pImuGb, GTGrabber *pGTGb, const bool bClahe):
        mpSLAM(pSLAM), mpImuGb(pImuGb), mpGTGb(pGTGb), mbClahe(bClahe)
{
    dR_last = 0;
    tR_last = 0;
    //std::cout<<"Construct img0Buf:"<<img0Buf.size()<<std::endl;
    //std::cout<<"ImageGrabber Construct img0Buf:"<<mpImuGb->imuBuf.size()<<std::endl;
  //  std::cout<<"Construct pSLAM:"<<unsigned(pSLAM->mnClientId)<<std::endl;
}
GTGrabber::GTGrabber(){

}
void GTGrabber::GrabGT(const geometry_msgs::PointStampedConstPtr &gt_msg)
{
  mBufMutex.lock();
//std::cout<<"gt_msg:"<<gt_msg->point.x<<std::endl;
  GTBuf.push(gt_msg);
  mBufMutex.unlock();
  return;
}
