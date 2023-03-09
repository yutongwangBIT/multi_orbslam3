#ifndef ORBSLAM3_ROS_SERVERNODE_H_
#define ORBSLAM3_ROS_SERVERNODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "Datatypes.h"
#include "ServerSystem.h"

class ServerNode
{
  public:
    ServerNode(std::string sensor, ros::NodeHandle Nh, ros::NodeHandle nhPrivate);
    ~ServerNode();
    void Save();

  protected:

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    ORB_SLAM3::eSensor mSensor;

    ORB_SLAM3::ServerSystem* ss;

    std::string voc_file_path;
    std::string name_of_node_;


};

#endif //ORBSLAM3_ROS_ServerNode_H_
