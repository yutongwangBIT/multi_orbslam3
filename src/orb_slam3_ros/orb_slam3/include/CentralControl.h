
#ifndef CENTRALCONTROL_H_
#define CENTRALCONTROL_H_


#include <mutex>

//ROS
#include <ros/ros.h>

#include "Datatypes.h"
#include "CameraModels/GeometricCamera.h"

//Thirdpary
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;

namespace ORB_SLAM3{
//forward decs
class ClientHandler;
class ClientSystem;
class KeyFrame;
//--------------

struct CentralControl
{
public:
    CentralControl(ros::NodeHandle Nh, ros::NodeHandle NhPrivate,
                   uint8_t ClientId,
                   eSystemState SysState,
                   ClientSystem* pCS = nullptr,
                   UniqueIdDispenser* pUID = nullptr,
                   g2o::Sim3 g2oS_wc_wm = g2o::Sim3()
            )
        : mNh(Nh), mNhPrivate(NhPrivate),
          mnClientId(ClientId),
          mpCS(pCS),
          mpUID(pUID),
          mbOptActive(false),
          mSysState(SysState),
          mbCommLock(false),mbMappingLock(false),mbPlaceRecLock(false),mbTrackingLock(false),
          mg2oS_wcurmap_wclientmap(g2oS_wc_wm),
          mbGotMerged(false),mbOptimized(false)
        {
            //...THIS IS FPR CLIENT
        }

    CentralControl(ros::NodeHandle Nh, ros::NodeHandle NhPrivate,
                   uint8_t ClientId,
                   eSystemState SysState,
                   GeometricCamera* Camera,
                   ClientHandler* pCH = nullptr,
                   UniqueIdDispenser* pUID = nullptr,
                   g2o::Sim3 g2oS_wc_wm = g2o::Sim3()
            )
        : mNh(Nh), mNhPrivate(NhPrivate),
          mnClientId(ClientId),
          mpCH(pCH),mpCamera(Camera),
          mpUID(pUID),
          mbOptActive(false),
          mSysState(SysState),
          mbCommLock(false),mbMappingLock(false),mbPlaceRecLock(false),mbTrackingLock(false),
          mg2oS_wcurmap_wclientmap(g2oS_wc_wm), mNearestKf(nullptr),
          mbGotMerged(false),mbOptimized(false)
        {
            //...THIS IS FOR SERVER
        }

    //ROS
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    //Infrastucture
    UniqueIdDispenser* mpUID;
    uint8_t mnClientId;
    string mNativeOdomFrame;
    ClientHandler* mpCH;
    ClientSystem* mpCS;
    g2o::Sim3 mg2oS_wcurmap_wclientmap; //Sim3 world client to world map
    eSystemState mSysState;
    //System Control
    bool mbOptActive;
    bool mbGotMerged;
    bool mbOptimized; //signalizes that has seen GBA;
    Eigen::Matrix4d mT_SC;

    GeometricCamera* mpCamera;
    //Thread Sync
    bool LockComm(){unique_lock<mutex> lock(mMutexComm); if(!mbCommLock){mbCommLock = true; return true;} else return false;}
    bool LockMapping(){unique_lock<mutex> lock(mMutexMapping); if(!mbMappingLock){mbMappingLock = true; return true;} else return false;}
    bool LockPlaceRec(){unique_lock<mutex> lock(mMutexPlaceRec); if(!mbPlaceRecLock){mbPlaceRecLock = true; return true;} else return false;}
    bool LockTracking(){unique_lock<mutex> lock(mMutexTracking); if(!mbTrackingLock){mbTrackingLock = true; return true;} else return false;}
    void UnLockComm(){unique_lock<mutex> lock(mMutexComm);if(mbCommLock){mbCommLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Comm -- was not locked" << endl; throw infrastructure_ex();}}
    void UnLockMapping(){unique_lock<mutex> lock(mMutexMapping);if(mbMappingLock){mbMappingLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Mapping -- was not locked" << endl; throw infrastructure_ex();}}
    void UnLockPlaceRec(){unique_lock<mutex> lock(mMutexPlaceRec);if(mbPlaceRecLock){mbPlaceRecLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock PlaceRec -- was not locked" << endl; throw infrastructure_ex();}}
    void UnLockTracking(){unique_lock<mutex> lock(mMutexTracking);if(mbTrackingLock){mbTrackingLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Tracking -- was not locked" << endl; throw infrastructure_ex();}}

    bool IsTrackingLocked(){unique_lock<mutex> lock(mMutexTracking); return mbTrackingLock;}

    void SetNearestKF(KeyFrame* nearest){unique_lock<mutex> lock(mMutexNearest); mNearestKf = nearest;}
    KeyFrame* GetNearestKF(){unique_lock<mutex> lock(mMutexNearest); if(mNearestKf){return mNearestKf;} else{return nullptr;}}

private:
    //Thread Sync
    bool mbCommLock;
    bool mbMappingLock;
    bool mbPlaceRecLock;
    bool mbTrackingLock;
    //Mutexes
    mutex mMutexComm;
    mutex mMutexMapping;
    mutex mMutexPlaceRec;
    mutex mMutexTracking;

    //JUST FOR VISUALIZING NEAREST KF
    KeyFrame* mNearestKf;
    mutex mMutexNearest;
};

} //end ns

#endif
