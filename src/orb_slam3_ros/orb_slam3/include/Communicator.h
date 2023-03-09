
#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_

//C++
#include <boost/bind.hpp>
#include <mutex>
#include <sstream>
#include <algorithm>

#include <time.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "ORBVocabulary.h"
#include "Datatypes.h"
#include "LocalMapping.h"

#include "CameraModels/GeometricCamera.h"
//Msgs
#include <orb_slam3_ros/KF.h>
#include <orb_slam3_ros/MP.h>
#include <orb_slam3_ros/KFred.h>
#include <orb_slam3_ros/MPred.h>
#include <orb_slam3_ros/Map.h>

using namespace std;
namespace ORB_SLAM3
{

//forward decs
class Atlas;
class LocalMapping;
class CentralControl;
class KeyFrameDatabase;
class KeyFrame;
class MapPoint;
//------------------

class Communicator
{
public:
    //---constructor---
    Communicator(CentralControl* pCC, ORBVocabulary* pVoc, Atlas* pAtlas, KeyFrameDatabase* pKFDB, eSensor Sensor=MONOCULAR, bool bLoadedMap = false);

    //---main---
    void RunClient();
    void RunServer();

    //---getter/setter---
    void SetLocalMapping(LocalMapping* pLocalMapping){mpLocalMapping = pLocalMapping;}
    /*void SetMapping(mappingptr pMapping) {mpMapping = pMapping;}
    void ChangeMap(mapptr pMap){mpMap = pMap;}
    void SetMapMatcher(matchptr pMatch) {mpMapMatcher = pMatch;}
    dbptr GetDbPtr(){return mpDatabase;}
    size_t GetClientId(){return mClientId;}
    idpair GetNearestKFid(){return mNearestKfId;}

    //---callbacks---
    void MapCbClient(ccmslam_msgs::MapConstPtr pMsg);
    void MapCbServer(ccmslam_msgs::MapConstPtr pMsg);
    void RequestReset();
    void RequestResetExternal();*/
    void MapSubsciberClient(orb_slam3_ros::MapConstPtr pMsg);
    void MapSubsciberServer(orb_slam3_ros::MapConstPtr pMsg);

    //--- data transfer---
    void PassKftoComm(KeyFrame* pKf, bool bIsIMUInitialized);
    void PassMptoComm(MapPoint* pMp, bool bIsIMUInitialized);
    void PassErasedKfIdtoComm(size_t mId, bool bIsIMUInitialized);
    void PassErasedMpIdtoComm(size_t mId, bool bIsIMUInitialized);
    void ErasedKfFromBuffer(KeyFrame* pKF);
   /* void DeleteMpFromBuffer(mpptr pMP);*/

    eSensor mSensor;
    GeometricCamera* mpCamera;
    uint8_t mnClientId;

    //void ChangeAtlas(Atlas* pAtlas);
protected:
    //---infrastructure---
    CentralControl* mpCC;
    Atlas* mpAtlas;
    KeyFrameDatabase* mpDatabase;
    ORBVocabulary* mpVoc;
    LocalMapping* mpLocalMapping;
   /* matchptr mpMapMatcher;*/

/*    const double mdPeriodicTime;*/
    bool mbLoadedMap = false; //indicates that map for this client was loaded from a file (only works for client 0)

   /* kfptr mpNearestKF; //client: store last nearest KF -- server: store current nearest KF
    idpair mNearestKfId;*/

    int mKfItBound;
    int mMpItBound;
    int mKfItBoundPub;
    int mMpItBoundPub;
    int CommRateClient;
    int mMaxMsgSentPerIt;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMap;
    ros::Subscriber mSubMap;

    string mSubKfTopicName;
    string mSubMpTopicName;
    int mPubMapBufferSize, mSubMapBufferSize;

    //---buffer checks---
    bool CheckBufferKfOut();
/*    bool CheckBufferKfIn();
    bool CheckBufferKfOut();
    bool CheckBufferMpIn();
    bool CheckBufferMpOut();*/

    //---publish/receive---
    void PublishMapServer();
    void PublishMapClient();
    double mdLastTimePub;
    void ProcessKfInServer();
    void ProcessKfInClient();
    void ProcessMpInServer();
    void ProcessMpInClient();
    void ProcessErasedMpInServer();
    void ProcessErasedKfInServer();
    KeyFrame* mpKFLastFront;

/*    size_t mMsgCountLastMapMsg;
    kfptr mpKFLastFront;
    size_t mnMaxKfIdSent;*/

    //---IO buffers---
    /*set<kfptr,kfcmp> mspBufferKfOut;
    set<mpptr,mpcmp> mspBufferMpOut;
    list<msgKFPair> mlBufKFin;
    list<msgMPPair> mlBufMPin;

    list<kfptr> mlpAddedKfs;

    set<size_t> msAcksKF;
    set<size_t> msAcksMP;
    void SetWeakAckKF(size_t id);
    void SetWeakAckMP(size_t id);
    size_t mnWeakAckKF;
    size_t mnWeakAckMP;
    list<AckPairKF> mlKfOpenAcks;
    list<AckPairMP> mlMpOpenAcks;

    //---Reset---
    void ResetIfRequested();
    void ResetCommunicator();
    void ResetDatabase();
    void ResetMap();
    void ResetMapping();
    void ResetMapMatcher();
    bool mbResetRequested;*/
    size_t mnMaxKfIdSent;
    list<pair<size_t,KeyFrame*>> mlKfOpenAcks;
    list<pair<size_t,MapPoint*>> mlMpOpenAcks;
    bool mbResetRequested;

    set<KeyFrame*> mspBufferKfOut;
    set<MapPoint*> mspBufferMpOut;
    vector<size_t> mspBufferErasedKfIdsOut;
    vector<size_t> mspBufferErasedMpIdsOut;
    list<pair<orb_slam3_ros::KF, orb_slam3_ros::KFred>> mlBufKFin;
    list<pair<orb_slam3_ros::MP, orb_slam3_ros::MPred>> mlBufMPin;
    list<size_t> mlBufErasedMPin;
    list<size_t> mlBufErasedKFin;
    pair<long unsigned int, uint8_t> mNearestKfId;
    KeyFrame* mpNearestKF;
    KeyFrame* mpLastNearestKF;

    //--mutexes
    mutex mMutexBuffersOut;
    mutex mMutexBuffersIn;
    mutex mMutexMsgBuffer;
    mutex mMutexReset;
    mutex mMutexLastMsgId;
    mutex mMutexNearestKf;
    mutex mMutexScale;

    //--Final BA if interrupted
    size_t mnEmptyMsgs;

    //monitoring //CHECKHERE
    size_t mOutMapCount;
    size_t mServerMapCount;
/*    void CheckOrdering(ccmslam_msgs::Map msgMap);
    void ShowBufferContent(bool bGetMutexBufferIn = false, bool bGetMutexBufferOut = false);*/

    bool mbIsFirstKfInComm;

    int globalkfcounter;

    //FOR Imu
    IMU::Calib *mpImuCalib;

    float mScale;
    float mScaleLast;
    cv::Mat mRgw;

    void ApplyScaledRotation();

};

} //end ns

#endif
