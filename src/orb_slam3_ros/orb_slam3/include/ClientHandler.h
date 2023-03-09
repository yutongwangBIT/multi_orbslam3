#ifndef CLIENTHANDLER_H_
#define CLIENTHANDLER_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <sstream>

//ROS
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>

//SLAM
#include "Datatypes.h" 
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"

#include "CameraModels/GeometricCamera.h"

//Thirdparty
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;

namespace ORB_SLAM3{

//forward decs
class Atlas;
class LocalMapping;
class LoopClosing;
class Communicator;
class CentralControl;
class ServerViewer;
//----------------


class ClientHandler
{
public:
    ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, ORBVocabulary* pVoc, KeyFrameDatabase* pDB, Atlas* pAtlas,
                    uint8_t ClientId, const eSensor sensor, ServerViewer* pViewer, UniqueIdDispenser* pUID, eSystemState SysState=eSystemState::SERVER);

    void InitializeThreads();
    //void ChangeAtlas(Atlas* pAtlas, g2o::Sim3 g2oS_wnewmap_wcurmap);
    //---getter/setter---
    /*void SetMapMatcher(matchptr pMatch);
    void ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap);
    commptr GetCommPtr(){return mpComm;}
    trackptr GetTrackPtr(){return mpTracking;}
    mappingptr GetMappingPtr(){return mpMapping;}
    dbptr GetDbPtr(){return mpKFDB;}
    vocptr GetVocPtr(){return mpVoc;}
    kfptr GetCurrentRefKFfromTracking();
    int GetNumKFsinLoopFinder();
    int GetNumKFsinMapMatcher();

    //---forwarding---
    void ClearCovGraph(size_t MapId);

    //---Agent side---
    void CamImgCb(sensor_msgs::ImageConstPtr pMsg);
    void Reset();

    //---Map Save/Load---
    void LoadMap(const string &path_name);
    void SaveMap(const string &path_name);
    bool mbLoadedMap = false;*/ //indicates that map for this client was loaded from a file (only works for client 0)

private:
    void InitializeCC();
    void InitializeServer();

    //infrastructure
    CentralControl* mpCC;
    Atlas* mpAtlas;
    KeyFrameDatabase* mpKFDB;
    ORBVocabulary* mpVoc;
    Communicator* mpComm;
    LocalMapping* mpLocalMapper;
    ServerViewer* mpViewer;

    //server only
    LoopClosing* mpLoopCloser;

    eSystemState mSysState;


    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    //threads
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptCommunicator;

    //data
    uint8_t mnClientId;
    g2o::Sim3 mg2oS_wcurmap_wclientmap; //transformation from map into client

    //reset
    bool mbReset;

    //mutexes
    mutex mMutexThreads;
    mutex mMutexReset;

    eSensor mSensor;
    GeometricCamera* mpCamera;

    string node_name;

    UniqueIdDispenser* mpUID;
};

} //end ns

#endif
