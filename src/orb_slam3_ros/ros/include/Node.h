/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM3_ROS_NODE_H_
#define ORBSLAM3_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <orb_slam2_ros/dynamic_reconfigureConfig.h>

//#include "orb_slam2_ros/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include "System.h"
#include "Converter.h"
#include "orb_slam3_ros/KF.h"
#include "orb_slam3_ros/KFs.h"


class Node
{
  public:
    Node (ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();
    void Init ();
    struct Color
    {
        unsigned char red;
        unsigned char green;
        unsigned char blue;
        inline constexpr Color():Color(0, 0, 0){}
        inline constexpr Color(unsigned char red, unsigned char green, unsigned char blue)
                            :red(red), green(green), blue(blue){}
    };

  protected:
    void Update ();
    ORB_SLAM3::System* orb_slam_;
    ros::Time current_frame_time_;

    std::string camera_info_topic_;

  private:
    void PublishMapPoints (std::vector<ORB_SLAM3::MapPoint*> map_points);
  //  void PublishAllMaps(std::vector<ORB_SLAM3::Map*> vmaps);
    void PublishAllMaps(std::vector<ORB_SLAM3::Map*> vmaps);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishRenderedImage (cv::Mat image);
    /*void PublishKeyFrames(std::vector<ORB_SLAM3::KeyFrame*> KFs);*/
    void PublishKeyFrames(orb_slam3_ros::KFs mKFs);
    //void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);
    //bool SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);
    void LoadOrbParameters (ORB_SLAM3::ORBParameters& parameters);

    tf::Transform TransformFromMat (cv::Mat position_mat);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points);
    pcl::PointCloud<pcl::PointXYZRGB> MapPointsToPointCloudWithColor (std::vector<ORB_SLAM3::MapPoint*> map_points, Color color);
    //dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher atlas_points_publisher_;
    ros::Publisher atlas_maps_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher keyframes_publisher_;

    //TODO DELETED
    ros::Publisher KeyFramesFromMsg_pub;
    visualization_msgs::MarkerArray KeyFramesFromMsg;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM3::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

    float mfFrameColors2[6][3] = {{0.0f, 0.0f, 0.5f},
                                {0.4f, 0.2f, 0.5f},
                                {0.5f, 0.1f, 0.2f},
                                {0.3f, 0.0f, 0.5f},
                                {0.5f, 0.5f, 0.0f},
                                {0.0f, 0.5f, 0.5f}};
};

#endif //ORBSLAM3_ROS_NODE_H_
