#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Datatypes.h"

#include <set>
#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <orb_slam3_ros/Map.h>

namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
//class Atlas;
class KeyFrameDatabase;
class CentralControl;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;
        // Set of KeyFrames and MapPoints, in this version the set serializator is not working
        //ar & mspKeyFrames;
        //ar & mspMapPoints;

        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;

        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnLastLoopKFid;
    }

public:
    Map();
    Map(int initKFid, uint8_t ClientId, eSystemState mSystem);
    Map(int initKFid, uint8_t ClientId, eSystemState mSystem, unsigned long int nMapId, CentralControl* pCC);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames(bool bSort=false);
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap(uint8_t nClientId);
    void SetStoredMap(uint8_t nClientId);
    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse(uint8_t nClientId);
    bool IsInUse();
    std::vector<int> CountClients();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void RotateMap(const cv::Mat &R);
    void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel=false, const bool bLockSend=false, const cv::Mat t=cv::Mat::zeros(cv::Size(1,3),CV_32F));

    void SetInertialSensor();
    bool IsInertial();
    void SetInertialBA1();
    void SetInertialBA2();
    bool GetInertialBA1();
    bool GetInertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    //CHANGED FOR COMMUNICATION
    KeyFrame* GetKeyFrameWithId(uint8_t nClientId,size_t nId);
    MapPoint* GetMapPointWithId(uint8_t nClientId,size_t nId);
    KeyFrame* GetPredecessor(KeyFrame* pKF);
    bool IsMpErased(uint8_t clientId, size_t mId);
    bool IsKfErased(uint8_t clientId, size_t mId);
    KeyFrame* GetErasedKF(uint8_t clientId, size_t mId);

    long unsigned int mnId;
    uint8_t mnClientId;
    long unsigned int GetMaxKFidUnique();
    void AddCCPtr(CentralControl* pCC);
    void EraseCCPtr(CentralControl* pCC);
    std::set<CentralControl*> GetCCPtrs();

    void PackVicinityToMsg(KeyFrame* kfcur, orb_slam3_ros::Map &mapMsg, CentralControl* pCC, uint8_t clientId);
    void PackVicinityToMsg2(KeyFrame* kfcur, orb_slam3_ros::Map &mapMsg, CentralControl* pCC, uint8_t clientId);

    void ConvertToMessage(orb_slam3_ros::Map &msgMap);
protected:

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    //CHANGD FOR COMMUNICATION
    std::map<idpair, KeyFrame*> mmpKeyFrames;
    std::map<idpair, KeyFrame*> mmpErasedKeyFrames;
    std::map<idpair, MapPoint*> mmpMapPoints;
    std::map<idpair, MapPoint*> mmpErasedMapPoints;
    eSystemState mSysState;

    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    long unsigned int mnLastLoopKFid;
    long unsigned int mnMaxKFidUnique;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    bool mIsInUse;
    std::map<uint8_t, bool> mmpIsInUse;
   // bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    std::mutex mMutexMap;
    //CHANGED
    std::mutex mMutexErase;

//    std::set<Atlas*> mspAtlas;
//    std::mutex mMutexAtlas;
    std::mutex mMutexInertial;
    std::mutex mMutexCC;
    std::set<CentralControl*> mspCC;

    //ADD SCALE AND RWG TO INFORM SERVER
    float mScale;
    cv::Mat mRgw;
};

} //namespace ORB_SLAM3

#endif // MAP_H
