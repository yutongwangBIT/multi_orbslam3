#ifndef CLIENTNODE_H_
#define CLIENTNODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ClientSystem.h"
#include "Datatypes.h"
#include "ImuTypes.h"
//#include "Graber.h"


class ClientNode
{
  public:
    ClientNode(string sensor, ros::NodeHandle Nh, ros::NodeHandle nhPrivate);
    ~ClientNode();
  protected:
    ORB_SLAM3::ClientSystem* client_system_;

    ros::Time current_frame_time_;

  private:
    void LoadOrbParameters(ORB_SLAM3::ORBParameters& parameters);

    //MONO
    void ImageCallbackMono(const sensor_msgs::ImageConstPtr& msg);
    ros::Subscriber image_subscriber;

    //STEREO
    void ImageCallbackStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *right_sub_;
    message_filters::Synchronizer<sync_pol> *sync_;

    //RGBD
    void ImageCallbackRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;


    std::string name_of_node_;

    //ROS infrastructure
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    ORB_SLAM3::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    double last_timestamp;
};

#endif //ORBSLAM3_ROS_NODE_H_
