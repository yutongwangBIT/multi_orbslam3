#ifndef MAPPOINT_H
#define MAPPOINT_H


#include "Converter.h"
#include "ORBmatcher.h"
#include "Datatypes.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

#include "orb_slam3_ros/MP.h"
#include "orb_slam3_ros/Map.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;
class Communicator;
class Atlas;

class MapPoint
{
    template<class Archive>
    void serializeMatrix(Archive &ar, cv::Mat& mat, const unsigned int version)
    {
        int cols, rows, type;
        bool continuous;

        if (Archive::is_saving::value) {
            cols = mat.cols; rows = mat.rows; type = mat.type();
            continuous = mat.isContinuous();
        }

        ar & cols & rows & type & continuous;
        if (Archive::is_loading::value)
            mat.create(rows, cols, type);

        if (continuous) {
            const unsigned int data_size = rows * cols * mat.elemSize();
            ar & boost::serialization::make_array(mat.ptr(), data_size);
        } else {
            const unsigned int row_size = cols*mat.elemSize();
            for (int i = 0; i < rows; i++) {
                ar & boost::serialization::make_array(mat.ptr(i), row_size);
            }
        }
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Variables used by the tracking
        ar & mTrackProjX;
        ar & mTrackProjY;
        ar & mTrackDepth;
        ar & mTrackDepthR;
        ar & mTrackProjXR;
        ar & mTrackProjYR;
        ar & mbTrackInView;
        ar & mbTrackInViewR;
        ar & mnTrackScaleLevel;
        ar & mnTrackScaleLevelR;
        ar & mTrackViewCos;
        ar & mTrackViewCosR;
        ar & mnTrackReferenceForFrame;
        ar & mnLastFrameSeen;

        // Variables used by local mapping
        ar & mnBALocalForKF;
        ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        ar & mnLoopPointForKF;
        ar & mnCorrectedByKF;
        ar & mnCorrectedReference;
        serializeMatrix(ar,mPosGBA,version);
        ar & mnBAGlobalForKF;
        ar & mnBALocalForMerge;
        serializeMatrix(ar,mPosMerge,version);
        serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        serializeMatrix(ar,mWorldPos,version);
        //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mNormalVector,version);
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;
        ar & mnVisible;
        ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    MapPoint();

    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap, Communicator* pComm, eSystemState SysState, size_t UniqueId, uint8_t ClientId);
    MapPoint(orb_slam3_ros::MP *pMsg, Map* pMap, Communicator* pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap, Communicator* pComm, eSystemState SysState, size_t UniqueId, uint8_t ClientId);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF, Communicator* pComm, eSystemState SysState, size_t UniqueId, uint8_t ClientId);

    void SetWorldPos(const cv::Mat &Pos, bool bLock=false, bool bLockSend=false);

    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,int idx);
    void EraseObservation(KeyFrame* pKF);

    std::tuple<int,int> GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void SetNormalVector(cv::Mat& normal);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void PrintObservations();

    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid);

    //CHANGED FOR COMMUNICATION
    orb_slam3_ros::MP GetMPMessage(KeyFrame* pRefKF=nullptr);
    void SendMe();
    void MarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = true;}
    void UnMarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = false;}
    //void EraseInOutBuffer();
    bool IsInOutBuffer() {unique_lock<mutex> lock(mMutexOut); return mbInOutBuffer;}
    bool IsSent(){unique_lock<mutex> lock(mMutexOut); return mbSentOnce;}
    bool IsFromServer(){unique_lock<mutex> lock(mMutexOut); return mbFromServer;} // That means I donnt want to send it again....(at this moment)
    void SetFromServer(){unique_lock<mutex> lock(mMutexOut); mbFromServer=true;}

    size_t GetMaxObsKFId(){unique_lock<mutex> lock(mMutexFeatures); return mMaxObsKFId;}
    void ConvertToMessage(orb_slam3_ros::Map &msgMap, KeyFrame* pRefKF);
    void EstablishInitialConnections();
    void UpdateFromMessage(orb_slam3_ros::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    void UpdateObservation(orb_slam3_ros::MP *pMsg, Atlas* pAtlas, bool bSetPose=false);

    int mInsertedWithKF;


public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    size_t mnUniqueId;
    uint8_t mnClientId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    cv::Mat mPosMerge;
    cv::Mat mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    KeyFrame* mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

protected:
    void WriteMembersFromMessage(orb_slam3_ros::MP* msg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(orb_slam3_ros::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap=g2o::Sim3());
    bool SetPoseFromMessage(orb_slam3_ros::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap=g2o::Sim3());
    void ReduceMessage(orb_slam3_ros::MP *pMsgFull, orb_slam3_ros::MPred *pMsgRed);
    bool mbPoseLock;
     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,std::tuple<int,int> > mObservations;
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;
     long unsigned int mBackupRefKFId;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     bool mbIsErasedFromMap;//add for flags to send
     MapPoint* mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;
     std::mutex mMutexOut;

     //CAHNGED FOR COMMUNICATION
    std::set<Communicator*> mspComm;
    eSystemState mSysState;

    size_t mMaxObsKFId;

    bool mbInOutBuffer;
    bool mbSentOnce;
    bool mbSendFull;
    bool mbPoseChanged;
    bool mbFromServer;


};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
