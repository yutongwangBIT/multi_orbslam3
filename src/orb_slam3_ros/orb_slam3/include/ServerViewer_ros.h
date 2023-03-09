#ifndef ORB_SLAM3_VIEWER_H_
#define ORB_SLAM3_VIEWER_H_


//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

//Msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
 
#include <mutex>
using namespace std;

namespace ORB_SLAM3{

//forward decs
class Atlas;
class Map;
class MapPoint;
class KeyFrame;
class CentralControl;
//----------

class ServerViewer
{
public:
    ServerViewer(Atlas* pAtlas0, Atlas* pAtlas1, Atlas* pAtlas2, Atlas* pAtlas3, CentralControl* pCC);
    void DrawActiveMap();
    void DrawNonActiveMap();
    //Main function
    void RunServer();


private:

    //ROS
    ros::NodeHandle mNh;

    //Infrastructure
    CentralControl* mpCC;
    Atlas* mpAtlas;
    std::vector<Atlas*> mvpAtlas;
    //std::map<int, std::vector<geometry_msgs::Pose>> mvpKFs;
    std::map<int, std::vector<std::pair<long unsigned int, geometry_msgs::Pose>>> mvpKFs;
    int count;
    std::map<int, std::vector<geometry_msgs::Point>> mvpMPs;
    //+++++++++++++++++++++++++++++++++
    //Map Visualization
    //+++++++++++++++++++++++++++++++++
    //Publisher
    ros::Publisher mPubMarker0;
    ros::Publisher mPubMarker1;
    ros::Publisher mPubMarker2;
    ros::Publisher mPubMarker3;
    ros::Publisher mPubNonActiveMap0;
    ros::Publisher mPubNonActiveMap1;
    ros::Publisher mPubNonActiveMap2;
    ros::Publisher mPubNonActiveMap3;
    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {1.0f, 0.0f, 0.0f},
                                {0.0f, 1.0f, 0.0f},
                                {1.0f, 1.0f, 0.0f},
                                {1.0f, 0.0f, 1.0f},
                                {0.0f, 1.0f, 1.0f}};

    float mfFrameColors2[6][3] = {{0.0f, 0.0f, 0.5f},
                                {0.4f, 0.2f, 0.5f},
                                {0.5f, 0.1f, 0.2f},
                                {0.3f, 0.0f, 0.5f},
                                {0.5f, 0.5f, 0.0f},
                                {0.0f, 0.5f, 0.5f}};

};

} //end namespace

#endif
