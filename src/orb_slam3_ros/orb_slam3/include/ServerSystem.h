#ifndef SERVERSYSTEM_H_
#define SERVERSYSTEM_H_

//C++
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "ImuTypes.h"
#include "Datatypes.h"
//CHANGED
#include <ros/ros.h>
#include <ros/time.h>

using namespace std;

namespace ORB_SLAM3{

class Atlas;
class ClientHandler;
class CentralControl;
class ServerViewer;
class MapDrawer;

class ServerSystem
{
public:
    ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const eSensor sensor);
    void InitializeClients();
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

private:
    void InitializeAtlas();
    void InitializeViewer();

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Atlas structure that stores the map pointers, which contain all KeyFrames and MapPoints.
    Atlas* mpAtlas0;
    Atlas* mpAtlas1;
    Atlas* mpAtlas2;
    Atlas* mpAtlas3;

    //ClientHandler manages each client
    ClientHandler* mpCH0;
    ClientHandler* mpCH1;
    ClientHandler* mpCH2;
    ClientHandler* mpCH3;

    //CentralControl for all
    CentralControl* mpCC;
    //Viewer
    ServerViewer* mpViewer;
    std::thread* mptViewer;
    MapDrawer* mpMapDrawer;
/*    MapDrawer* mpMapDrawer0;
    MapDrawer* mpMapDrawer1;
    MapDrawer* mpMapDrawer2;
    MapDrawer* mpMapDrawer3;*/

    //ROS infrastructure
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    int mNumOfClients;
    int mMaxClients;

    UniqueIdDispenser* mpUID;

};

} //end namespace

#endif
