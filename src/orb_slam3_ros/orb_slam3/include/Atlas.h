#ifndef ATLAS_H
#define ATLAS_H


#include "Datatypes.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "CameraModels/GeometricCamera.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"
#include "Communicator.h"
#include "CentralControl.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


namespace ORB_SLAM3
{
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;
class Communicator;
class CentralControl;

//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

class Atlas
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        //ar.template register_type<Pinhole>();
        //ar.template register_type<KannalaBrandt8>();

        // Save/load the set of maps, the set is broken in libboost 1.58 for ubuntu 16.04
        //ar & mspMaps;
        ar & mvpBackupMaps;
        ar & mvpCameras;
        //ar & mvpBackupCamPin;
        //ar & mvpBackupCamKan;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
        ar & Map::nNextId;
        ar & Frame::nNextId;
        ar & KeyFrame::nNextId;
        ar & MapPoint::nNextId;
        ar & GeometricCamera::nNextId;
        ar & mnLastInitKFidMap;
    }

public:
    Atlas(uint8_t ClientId, eSystemState SysState);
    Atlas(int initKFid, uint8_t ClientId, eSystemState SysState); // When its initialization the first map is created
    ~Atlas();

    void CreateNewMap();
    void ChangeMap(Map* pMap);

    unsigned long int GetLastInitKFid();


    // Method for change components in the current map
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    //void EraseMapPoint(MapPoint* pMP);
    //void EraseKeyFrame(KeyFrame* pKF);

    void AddCamera(GeometricCamera* pCam);

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    // Method for get data in current map
    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    KeyFrame* GetKeyFrameWithId(uint8_t nClientId,size_t nId);
    MapPoint* GetMapPointWithId(uint8_t nClientId,size_t nId);

    vector<Map*> GetAllMaps();

    int CountMaps();

    void clearMap();

    void clearAtlas();

    Map* GetCurrentMap();

    void SetMapBad(Map* pMap);
    void RemoveBadMaps();

    bool isInertial();
    void SetInertialSensor();
    void SetImuInitialized();
    bool isImuInitialized();
    //ADDED
    void SetInertialBA1();
    void SetInertialBA2();

    // Function for garantee the correction of serialization of this object
    void PreSave();
    void PostLoad();

    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
    KeyFrameDatabase* GetKeyFrameDatabase();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    ORBVocabulary* GetORBVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();

    //CHANGED FOR COMMUNICATION
    void SetCommunicator(Communicator* pComm) {unique_lock<mutex> lock(mMutexComm); mspComm.insert(pComm);}
    std::set<Communicator*> GetCommunicator() {unique_lock<mutex> lock(mMutexComm); return mspComm;}

  //  bool LockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate); if(!mbLockMapUpdate){mbLockMapUpdate = true; return true;} else return false;}
  //  void UnLockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate);if(mbLockMapUpdate){mbLockMapUpdate = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock MapUpdate -- was not locked" << endl; throw infrastructure_ex();}}

  //  bool LockViewer(){unique_lock<mutex> lock(mMutexViewer); if(!mbLockViewer){mbLockViewer = true; return true;} else return false;}
  //  void UnLockViewer(){unique_lock<mutex> lock(mMutexViewer);if(mbLockViewer){mbLockViewer = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock MapUpdate -- was not locked" << endl; throw infrastructure_ex();}}

    long int GetMaxOriginMapId(uint8_t nClientId);
    void CreateNewMapServer(unsigned long int mnInitKFId, unsigned long int mapId, uint8_t nClientId);
    //void InsertMap(Map* pMap);
    uint8_t mnClientId;
    string mOdomFrame;
/*    void AddMapToSearchKF(std::make_pair<uint8_t, long unsigned int> pair_kf_ids, Map* pMap);
    void SearchKFInMap(std::make_pair<uint8_t, long unsigned int> pair_kf_ids);
*/
    void SetCC(CentralControl* pCC){unique_lock<mutex> lock(mMutexAtlas); mpCC=pCC;}
    CentralControl* GetCC(){unique_lock<mutex> lock(mMutexAtlas); return mpCC;}
    std::vector<int> GetMapCountClients();


protected:

    std::set<Map*> mspMaps;
    std::set<Map*> mspBadMaps;
    // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
    std::vector<Map*> mvpBackupMaps;
    Map* mpCurrentMap;

    std::vector<GeometricCamera*> mvpCameras;
    std::vector<KannalaBrandt8*> mvpBackupCamKan;
    std::vector<Pinhole*> mvpBackupCamPin;

    //Pinhole testCam;
    std::mutex mMutexAtlas;

    unsigned long int mnLastInitKFidMap;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    //CHANGED FOR COMMUNICATION
    //std::set<CentralControl*> mspCC;
    CentralControl* mpCC;
    std::set<Communicator*> mspComm;

    eSystemState mSysState;

  /*  std::mutex mMutexMapUpdate;
    bool mbLockMapUpdate;*/
  //  std::mutex mMutexViewer;
  //  bool mbLockViewer;

    unsigned int mnMaxOriginMapId;
    std::map<int, long int> mmpMaxOriginMapIds;

/*    std::map<std::pair<uint8_t, long unsigned int>, Map*> mmpMap;
    std::mutex mMutexmmpMap;*/
    std::mutex mMutexComm;
    std::mutex mMutexCC;

}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
