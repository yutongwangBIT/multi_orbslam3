#include "ServerNode.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Server");
  ros::start();

  if(argc > 2) {
      ROS_WARN ("Arguments supplied via command line are neglected.");
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ros::NodeHandle Nh;
  ros::NodeHandle nhPrivate;

  ServerNode sn(std::string(argv[1]), Nh, nhPrivate);

  ROS_INFO("started server node...");
  ros::spin();
  ros::shutdown();
  sn.Save();
    /*ROS_INFO("started CSLAM server node...");

    ros::MultiThreadedSpinner MSpin(2);

    MSpin.spin();

    ros::waitForShutdown();*/
   /* ros::Rate r(2);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }*/

    return 0;
}

ServerNode::ServerNode(std::string sensor, ros::NodeHandle Nh, ros::NodeHandle nhPrivate): nh_(Nh), nhPrivate_(nhPrivate)
{
  name_of_node_ = ros::this_node::getName();

  if(sensor=="MONO")
    mSensor = ORB_SLAM3::MONOCULAR;
  else if(sensor=="IMU_MONOCULAR")
    mSensor = ORB_SLAM3::IMU_MONOCULAR;
  else if(sensor=="STEREO")
    mSensor = ORB_SLAM3::STEREO;
  else if(sensor=="RGBD")
    mSensor = ORB_SLAM3::RGBD;
  else
    ROS_WARN ("The types are: MONO, STEREO or RGBD");

  //Load Vocabunary
  nh_.param<std::string>(name_of_node_ + "/voc_file", voc_file_path, "file_not_set");

  //construct server system
   ss = new ORB_SLAM3::ServerSystem(nh_, nhPrivate_, voc_file_path, mSensor);
  //Initialize ClientHandlers for Server, it also starts the subscribers
  ss->InitializeClients();
}
void ServerNode::Save(){
    std::string file_path;
    nh_.param<std::string>(name_of_node_ + "/save_path", file_path, "file_not_set");
    ss->SaveKeyFrameTrajectoryEuRoC(file_path);
}

ServerNode::~ServerNode () {
}
