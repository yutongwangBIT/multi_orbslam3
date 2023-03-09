#ifndef RGBDINERTIALNODE_H_
#define RGBDINERTIALNODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <XmlRpcException.h>

#include "ClientSystem.h"
#include "Datatypes.h"
#include "ImuTypes.h"

class ImuGrabber
{
public:
    ImuGrabber();
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::ClientSystem* pSLAM, ImuGrabber *pImuGb, const bool bClahe);

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg, const string typeName);
    void rgbdSyncWithImu();
    void ImageCallbackRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    queue<std::pair<sensor_msgs::ImageConstPtr,sensor_msgs::ImageConstPtr>> rgbdBuf;
    std::mutex mBufMutex;

    ORB_SLAM3::ClientSystem* mpSLAM;
    ImuGrabber *mpImuGb;

    double tR_last;
    float dR_last;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif //ORBSLAM3_ROS_NODE_H_
