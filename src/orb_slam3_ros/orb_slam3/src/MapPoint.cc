

#include "MapPoint.h"
#include "Communicator.h"


#include<mutex>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint():
    mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false), mMaxObsKFId(0),
    mpReplaced(static_cast<MapPoint*>(NULL))
{
    mpReplaced = static_cast<MapPoint*>(NULL);
}

MapPoint::MapPoint(orb_slam3_ros::MP *pMsg, Map* pMap, Communicator* pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 mg2oS_wcurmap_wclientmap)
    : nObs(0),mpReplaced(static_cast<MapPoint*>(NULL)),mpMap(pMap), /*mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0),
      mnCorrectedByKF(0), mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1),*/
      mnUniqueId(UniqueId), mfMinDistance(pMsg->mfMinDistance), mfMaxDistance(pMsg->mfMaxDistance),
      mbPoseChanged(false), mbSentOnce(true),mbInOutBuffer(false),mnOriginMapId(pMsg->mnOriginMapId), mInsertedWithKF(-1),
      mSysState(SysState), mbBad(false), mMaxObsKFId(0),mbSendFull(true), mbPoseLock(false) //TODO: there might be other parameters should be initialized here
{
    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;
         throw infrastructure_ex();
    }
    mspComm.insert(pComm);
    if(mSysState == eSystemState::CLIENT)
        mbFromServer = true;
    WriteMembersFromMessage(pMsg, mg2oS_wcurmap_wclientmap);
}

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap, Communicator* pComm, eSystemState SysState, size_t UniqueId, uint8_t ClientId):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0), mSysState(SysState), mnClientId(ClientId),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mMaxObsKFId(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false), mbIsErasedFromMap(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mbSentOnce(false), mbPoseChanged(false),
    mnOriginMapId(pMap->GetId()), mbSendFull(true), mbPoseLock(false), mnUniqueId(UniqueId), mbFromServer(false)
{
    mspComm.insert(pComm);
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap, Communicator* pComm,
    eSystemState SysState, size_t UniqueId, uint8_t ClientId):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0), mSysState(SysState), mnClientId(ClientId),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mMaxObsKFId(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false), mbIsErasedFromMap(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mbSentOnce(false), mbPoseChanged(false),
    mnOriginMapId(pMap->GetId()), mbSendFull(true), mbPoseLock(false), mnUniqueId(UniqueId),mbFromServer(false)
{
    mspComm.insert(pComm);
    mInvDepth=invDepth;
    mInitU=(double)uv_init.x;
    mInitV=(double)uv_init.y;
    mpHostKF = pHostKF;

    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF, Communicator* pComm, eSystemState SysState, size_t UniqueId, uint8_t ClientId):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mSysState(SysState), mnClientId(ClientId),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0), mMaxObsKFId(0), mnUniqueId(UniqueId),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mbSendFull(true),
    mnFound(1), mbBad(false), mbIsErasedFromMap(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId()),
    mbSentOnce(false), mbPoseChanged(false), mbPoseLock(false),mbFromServer(false)
{
    mspComm.insert(pComm);
    Pos.copyTo(mWorldPos);
    cv::Mat Ow;
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        cv::Mat Rwl = pFrame -> mRwc;
        cv::Mat tlr = pFrame -> mTlr.col(3);
        cv::Mat twl = pFrame -> mOw;

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::EstablishInitialConnections()
{
    //this is necessary, because we cannot use shared_from_this() in constructor
    if(mbBad){ //added by tongjiang
        std::cout<<"MapPoint is Bad~"<<std::endl;
        return;
    }
    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit = mObservations.begin();mit != mObservations.end();)
    {
        KeyFrame* pKFi = mit->first;
        int idx = get<0>(mit->second); //TODO FOR STEREO OR RGBD

        if(pKFi)
        {
            MapPoint* pMP = pKFi->GetMapPoint(idx);
            if(pMP)
            {
                //there is already an associated MP to this feature
                if(mnId == pMP->mnId && mnClientId == pMP->mnClientId)
                {
                    //But it's this MP, and we're calling the constructor -- thats inconsistent
                    std::cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Constructing MP & is already associated to KF" << std::endl;
                    std::cout << "KF-ID: " << pKFi->mnId <<" --- this MP-ID: " << mnId<<" --- comp. MP-ID: " << pMP->mnId<< std::endl;
                    throw infrastructure_ex();
                }
               /* else if(!pKFi->IsMpLocked(idx))
                {
                        pKFi->ReplaceMapPointMatch(idx,shared_from_this(),false); //WE HAVE THIS ALREADY....BUT IS THE LOCK NECESSARY?
                        pMP->EraseObservation(pKFi);
                    }*/ //WE DON'T HAVE THE LOCKED MAPPOINT LIST NOW
                else
                {
                    pKFi->ReplaceMapPointMatch(idx,this); //TODO: MAYBE A LOCK IS NECESSARY
                    pMP->EraseObservation(pKFi);
                        //its locked - cannot replace
                        //delete KF association
                  /*  mit = mObservations.erase(mit);
                    nObs--;
                    continue;*/
                   /* std::cout<<pMP->mnId<<"-th MP is already associated to this feature"<<std::endl;*/
                }
            }
            else
            {
                //nothing associated to this feature, we can add
                pKFi->AddMapPoint(this,idx);
            }
        }
        else
        {
            std::cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pKFi is NULLPTR" << std::endl;
        }

        ++mit;
    }
    mpRefKF = mObservations.begin()->first; //should not be bad, we checked that when we added the KF //WHY IT IS THE FIRST OBSERVATION

    if(mpRefKF->isBad())
    {
        std::cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << std::endl;
    }
}

void MapPoint::SetWorldPos(const cv::Mat &Pos, bool bLock, bool bLockSend)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT){
        std::cout<<"MP: mbPoseLock"<<std::endl;
        //return;
    }
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);


        //CAHNGED FOR COMMUNICATION
        if(mSysState == CLIENT){
            if(IsSent() && !bLockSend && !mbFromServer){
                mbPoseChanged = true;
                SendMe();
            }
        }
        else{
            if(bLock)
            {
                mbPoseLock = true;
            }
        }
    }
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int,int> indexes;
    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }
    if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }
    mObservations[pKF]=indexes;

    if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
/*    std::cout<<"pkf id:"<<pKF->mnId<<" added observation to:"<<nObs<<" on MapPoint id:"<<mnId<<std::endl;*/

    //CHANGED FOR COMMUNICATION
    if(mSysState == eSystemState::CLIENT)
    {
        if(pKF->mnId > mMaxObsKFId && pKF->mnClientId == mnClientId)
            mMaxObsKFId = pKF->mnId;

    }
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            //int idx = mObservations[pKF];
            tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(pKF);
            //CHANGED FOR COMMUNICATION
            if(mSysState == eSystemState::CLIENT)
            {
                if(pKF->mnId == mMaxObsKFId && pKF->mnClientId == mnClientId)
                {
                    mMaxObsKFId = 0;
                    for(map<KeyFrame*,tuple<int,int>>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
                    {
                        if(pKF->mnId > mMaxObsKFId && pKF->mnClientId == mnClientId)
                            mMaxObsKFId = pKF->mnId;
                    }
                }
            }

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}


std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*, tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*, tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        int leftIndex = get<0>(mit -> second), rightIndex = get<1>(mit -> second);
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }
    if(mSysState==CLIENT){
        mbIsErasedFromMap = true;
        SendMe();
    }
    else{
        //std::cout<<"A mappoint is erasing by server since it is bad"<<std::endl;
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(!pMP->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    /*unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);*/

    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,tuple<int,int>> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad()){
            tuple<int,int> indexes = mit -> second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            if(rightIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return tuple<int,int>(-1,-1);
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,tuple<int,int>> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }
    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(leftIndex != -1){
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }
        if(rightIndex != -1){
            cv::Mat Owi = pKF->GetRightCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }
    }
    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);

    tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> NLeft == -1){
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if(leftIndex != -1){
        level = pRefKF -> mvKeys[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
    }
    //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

void MapPoint::SetNormalVector(cv::Mat& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

void MapPoint::PrintObservations()
{
    cout << "MP_OBS: MP " << mnId << endl;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        tuple<int,int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
    }
}

Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void MapPoint::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP)
{
    mBackupReplacedId = -1;
    if(mpReplaced && spMP.find(mpReplaced) != spMP.end())
        mBackupReplacedId = mpReplaced->mnId;

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
    // Save the id and position in each KF who view it
    for(std::map<KeyFrame*,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
    {
        KeyFrame* pKFi = it->first;
        if(spKF.find(pKFi) != spKF.end())
        {
            mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
            mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
        }
        else
        {
            EraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    if(spKF.find(mpRefKF) != spKF.end())
    {
        mBackupRefKFId = mpRefKF->mnId;
    }
}

void MapPoint::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid)
{
    mpRefKF = mpKFid[mBackupRefKFId];
    if(!mpRefKF)
    {
        cout << "MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    mpReplaced = static_cast<MapPoint*>(NULL);
    if(mBackupReplacedId>=0)
    {
       map<long unsigned int, MapPoint*>::iterator it = mpMPid.find(mBackupReplacedId);
       if (it != mpMPid.end())
        mpReplaced = it->second;
    }

    mObservations.clear();

    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
        std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
        if(pKFi)
        {
           mObservations[pKFi] = indexes;
        }
    }

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
}



orb_slam3_ros::MP MapPoint::GetMPMessage(KeyFrame* pRefKF){
    orb_slam3_ros::MP Msg;
    Msg.mnId = static_cast<uint32_t>(mnId);
    Msg.mnUniqueId = static_cast<uint32_t>(mnUniqueId);
    Msg.mnClientId = static_cast<uint8_t>(mnClientId);
    Msg.mnOriginMapId = static_cast<uint16_t>(mnOriginMapId);
    Msg.mnFirstKFid = static_cast<uint16_t>(mnFirstKFid);
    Msg.mbBad = mbBad;
    Msg.mfMinDistance = mfMinDistance;
    Msg.mfMaxDistance = mfMaxDistance;
    Msg.mpRefKFId = static_cast<uint16_t>(mpRefKF->mnId);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);
    //Normal Vector
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);
    //Descriptor
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);
    //Observation
    for(std::map<KeyFrame*, std::tuple<int, int>>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
    {
        KeyFrame* pKFi = mit->first;
        if(!pKFi->isBad())
        {
            Msg.mObservations_KFIDs.push_back(static_cast<uint16_t>(pKFi->mnId));
            Msg.mObservations_KFClientIDs.push_back(static_cast<uint16_t>(pKFi->mnClientId));
            Msg.mObservations_n.push_back(static_cast<uint16_t>(get<0>(mit->second)));//get 0 is only for monnocular, if stereo, tuple 0 and 1.
            if(mSysState == eSystemState::SERVER){
                cv::Mat mTcref = pKFi->GetPose() * pRefKF->GetPoseInverse(); //TO CHECK
                orb_slam3_ros::RefPosKF refPose;
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::RefPosKF::_refpose_type, float>(mTcref, refPose.refpose);
                Msg.mObservations_KFs_RefPose.push_back(refPose);
            }
        }
    }
    Msg.mbSentOnce = mbSentOnce;
    if(mSysState == eSystemState::CLIENT){
        mbSentOnce = true;
        mbPoseChanged = false;
        mbSendFull = false;
    }
    return Msg;
}

void MapPoint::ConvertToMessage(orb_slam3_ros::Map &msgMap, KeyFrame* pRefKf)
{
    unique_lock<mutex> lockOut(mMutexOut);

    if(!mpRefKF){
        mpRefKF = mObservations.begin()->first;
        if(!mpRefKF){
            std::cout<<"REALLY NO PARENT"<<std::endl;
            mbBad = true;
            //SetBadFlag();
            return;
        }
    }
    if(mbBad)
        return;

    if(mbFromServer && mSysState == eSystemState::CLIENT)
        cout<<"WHYWHYWHYWHY?????????"<<endl;
    if(mbSendFull)
    {
        orb_slam3_ros::MP Msg;

        unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
        unique_lock<mutex> lockPos(mMutexPos,defer_lock);

        lock(lockFeat,lockPos);

        Msg = GetMPMessage(pRefKf);
        //USE RELATIVE POSE
        /*cv::Mat Rcw = pRefKf->GetPose().rowRange(0,3).colRange(0,3);
        cv::Mat tcw = pRefKf->GetPose().rowRange(0,3).col(3);*/
        cv::Mat Rcw = pRefKf->GetRotation();
        cv::Mat tcw = pRefKf->GetTranslation();
        cv::Mat RefPos = Rcw * mWorldPos + tcw;
        Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);
        Msg.mpPredKFId = static_cast<uint16_t>(pRefKf->mnId);
        Msg.mpPredKFClientId = static_cast<uint8_t>(pRefKf->mnClientId);


        if(mpRefKF)
        {
            Msg.mpParKFId = static_cast<uint16_t>(mpRefKF->mnId);
            Msg.mpParKFClientId = static_cast<uint8_t>(mpRefKF->mnClientId);

            cv::Mat Rparcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tparcw = mpRefKF->GetPose().rowRange(0,3).col(3);

            cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
        }
        else
        {
            Msg.mpParKFId = KFRANGE;
            Msg.mpParKFClientId = MAPRANGE;
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << "SENDFULL no Parent" << endl;
        }
        msgMap.MapPoints.push_back(Msg);
    }
    else
    {
        orb_slam3_ros::MPred Msg;

        unique_lock<mutex> lockPos(mMutexPos);

        /*if(mbPoseChanged || this->mbMultiUse) */// this part should only be called on client. System will send MP without pose change to server to inform it about a "MultiUse-Event"
        //mbMultiUse will be used , if the mappoint comes from another client. SetMultiUse will be called by tracker and it sends this mappoint out
        if(mbPoseChanged)
        {
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);
            mbPoseChanged = false;
            //USE RELATIVE POSE
            Msg.mpPredKFId = static_cast<uint16_t>(pRefKf->mnId);
            Msg.mpPredKFClientId = static_cast<uint8_t>(pRefKf->mnClientId);

            cv::Mat Rcw = pRefKf->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pRefKf->GetPose().rowRange(0,3).col(3);

            cv::Mat RefPos = Rcw * mWorldPos + tcw;

            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

            if(mpRefKF)
            {
                Msg.mpParKFId = static_cast<uint16_t>(mpRefKF->mnId);
                Msg.mpParKFClientId = static_cast<uint8_t>(mpRefKF->mnClientId);

                cv::Mat Rparcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
                cv::Mat tparcw = mpRefKF->GetPose().rowRange(0,3).col(3);

                cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
            }
            else
            {
                Msg.mpParKFId = KFRANGE;
                Msg.mpParKFClientId = MAPRANGE;
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
            }
            Msg.mbBad = mbBad;
            Msg.mnId = static_cast<uint32_t>(mnId);
            Msg.mnClientId = static_cast<uint8_t>(mnClientId);
            Msg.mnOriginMapId = static_cast<uint16_t>(mnOriginMapId);
            msgMap.MPUpdates.push_back(Msg);
        }
    }
}

void MapPoint::SendMe() //now only work for client
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
        cout << "bad?: " << (mbBad == true) << endl;
        return;
    }

    if(IsSent() && !IsInOutBuffer() && !mspComm.empty())
    {
        for(set<Communicator*>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            Communicator* pComm = *sit;
            if(mbIsErasedFromMap && pComm->mnClientId==mnClientId){
                pComm->PassErasedMpIdtoComm(mnId, mpMap->GetInertialBA1());
                return;
            }
            if(pComm==nullptr)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptr" << endl;
                return;
            }
            else{
                pComm->PassMptoComm(this, mpMap->GetInertialBA1());
                //pComm->PassMptoComm(this, mpMap->isImuInitialized());
            }
        }
    }
}
void MapPoint::UpdateObservation(orb_slam3_ros::MP *pMsg, Atlas *pAtlas, bool bSetPose){
    if(mSysState == eSystemState::CLIENT){
        //std::cout<<"before update, observation size:"<<mObservations.size()<<std::endl;
        mObservations.clear(); //rebuild observations sothat the kf connection can be updated
    }
    //CHECK REF PKF FOR THIS MP
    KeyFrame* pRefKF = mpMap->GetKeyFrameWithId(pMsg->mpPredKFClientId, pMsg->mpPredKFId);
    if(!pRefKF)
    {
        std::cout<<"COMING MP HAS NO REF KF"<<std::endl;
        return;
    }
    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
    {
        KeyFrame* pKFi = mpMap->GetKeyFrameWithId(pMsg->mObservations_KFClientIDs[idx], pMsg->mObservations_KFIDs[idx]);

        if(pKFi && !pKFi->isBad())
        {
            tuple<int,int> indexes;
            get<0>(indexes) = pMsg->mObservations_n[idx];//get 0 is only for monnocular, if stereo, tuple 0 and 1.
            mObservations[pKFi]=indexes;
            ++nObs;
        }
        /*else if(!pKFi)
        {
            pKFi = pAtlas->GetKeyFrameWithId(pMsg->mObservations_KFClientIDs[idx], pMsg->mObservations_KFIDs[idx]);
        */    /*if(pKFi && !pKFi->isBad()){
                std::cout<<"observed kf is in an old map"<<std::endl;
                pKFi->SetFromServer();
                //RECALL AND UPDATE THE PKFI
                cv::Mat matRef = cv::Mat(4,4,5);
                Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::RefPosKF::_refpose_type,float>
                                            (matRef, pMsg->mObservations_KFs_RefPose[idx].refpose);
                //pKFi->SetPose(matRef*pRefKF->GetPose());
                pKFi->UpdateMap(mpMap);
                mpMap->AddKeyFrame(pKFi);

                //TODO:pkf mPrevKf mNextKF? Does it really matter?
                tuple<int,int> indexes;
                get<0>(indexes) = pMsg->mObservations_n[idx];//get 0 is only for monnocular, if stereo, tuple 0 and 1.
                mObservations[pKFi]=indexes;
                ++nObs;
            }
            else
                std::cout<<"observed kf not found in an old map"<<std::endl;*/
        //}
    }

    if(nObs == 0)
    {
        std::cout<<"mbBad"<<std::endl;
        mbBad = true;
        return;
    }
    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit = mObservations.begin();mit != mObservations.end();)
    {
        KeyFrame* pKFi = mit->first;
        int idx = get<0>(mit->second); //TODO FOR STEREO OR RGBD

        if(pKFi)
        {
            MapPoint* pMP = pKFi->GetMapPoint(idx);
            if(pMP)
            {
                //there is already an associated MP to this feature
                if(mnId == pMP->mnId && mnClientId == pMP->mnClientId)
                {
                    //DO NOTHING
                }
                else
                {
                    pKFi->ReplaceMapPointMatch(idx,this); //TODO: MAYBE A LOCK IS NECESSARY
                    pMP->EraseObservation(pKFi);
                }
            }
            else
            {
                //nothing associated to this feature, we can add
                pKFi->AddMapPoint(this,idx);
            }
        }
        else
        {
            std::cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pKFi is NULLPTR" << std::endl;
        }

        ++mit;
    }
    mpRefKF = mObservations.begin()->first; //should not be bad, we checked that when we added the KF //WHY IT IS THE FIRST OBSERVATION

    if(!mpRefKF || mpRefKF->isBad())
    {
        std::cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << std::endl;
    }
    //std::cout<<"after update, observation size:"<<mObservations.size()<<std::endl;

    if(bSetPose){
        bSetPose = SetPoseFromMessage(pMsg);
        if(!bSetPose){
            mObservations.clear();
            mbBad = true;
            return;
        }
    }
}
void MapPoint::WriteMembersFromMessage(orb_slam3_ros::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    unique_lock<mutex> lockFeat(mMutexFeatures);
    mbSentOnce=true;

    mnId = pMsg->mnId;
    mnClientId = pMsg->mnClientId;
    mnFirstKFid = pMsg->mnFirstKFid;
    /*if(mnUniqueId==-100){ //if this mp is from server, its mnid should be uniqueid ?????
        mnUniqueId = pMsg->mnUniqueId;
        mnId = mnUniqueId;
    }
*/
    int nNotFound=0;
    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
    {
        KeyFrame* pKFi = mpMap->GetKeyFrameWithId(pMsg->mObservations_KFClientIDs[idx], pMsg->mObservations_KFIDs[idx]);

        if(pKFi && !pKFi->isBad())
        {
            tuple<int,int> indexes;
            get<0>(indexes) = pMsg->mObservations_n[idx];//get 0 is only for monnocular, if stereo, tuple 0 and 1.
            mObservations[pKFi]=indexes;
            ++nObs;
        }
        else{
            ++nNotFound;
        }
    }
    //std::cout<<"number of not found kf:"<<nNotFound<<",found:"<<nObs<<std::endl;
    if(nObs == 0)
    {
        mbBad = true;
        return;
    }

    mNormalVector = cv::Mat(3,1,5);
    Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::MP::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);
    mDescriptor = cv::Mat(1,32,0);
    Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::MP::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);

    mnFound = nObs;
    mnVisible = nObs;

    //if(mSysState == eSystemState::SERVER)
    //{
        bool bSetPos = SetPoseFromMessage(pMsg, mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mObservations.clear();
            mbBad = true;
            return;
        }
    //}
}

void MapPoint::UpdateFromMessage(orb_slam3_ros::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
   // mbOmitSending = true;

    unique_lock<mutex> lockOut(mMutexOut);
    cv::Mat mWorldPos_ = cv::Mat(3,1,5);

 /*   if(!mbPoseLock)
        {
        }
    mbOmitSending = false;*/ //TODO
    if(!pMsg->mbBad){
       /* if(mSysState == eSystemState::SERVER)
        {
            Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::MPred::_mWorldPos_type,float>(mWorldPos_,pMsg->mWorldPos);
            SetWorldPos(mWorldPos_);
           // std::cout<<"MapPoint UPDATED"<<std::endl;
        }*/
        if(mSysState == eSystemState::SERVER)
        {
            if(!mbPoseLock)
            {
                bool bSetPos =SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

                if(!bSetPos)
                {
                   /* mbOmitSending = false;*/
                    mbBad = true;//Added by TONGJIANG
                    return;
                }
            }
        }
    }
}


bool MapPoint::SetPoseFromMessage(orb_slam3_ros::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    orb_slam3_ros::MPred *pMsgRed = new orb_slam3_ros::MPred();

    ReduceMessage(pMsg,pMsgRed);

    bool bReturn = SetPoseFromMessage(pMsgRed, mg2oS_wcurmap_wclientmap);

    delete pMsgRed;

    return bReturn;
}

bool MapPoint::SetPoseFromMessage(orb_slam3_ros::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState == eSystemState::SERVER)
    {
        cv::Mat P3D_ref = cv::Mat(3,1,5); //in world client

        KeyFrame* pRef = mpMap->GetKeyFrameWithId(pMsg->mpPredKFClientId, pMsg->mpPredKFId);

        if(pRef)
        {
            Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::MP::_mPosPred_type,float>(P3D_ref,pMsg->mPosPred);
        }

        if(!pRef)
        {
            pRef = mpMap->GetKeyFrameWithId(pMsg->mpParKFClientId, pMsg->mpParKFId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::MP::_mPosPar_type,float>(P3D_ref,pMsg->mPosPar);
            }
        }

        if(!pRef)
        {
            return false;
        }

        float s = static_cast<double>(mg2oS_wcurmap_wclientmap.scale());
        s = 1/s;

        P3D_ref *=(1./s);

        if(!pRef->isBad())
        {
            cv::Mat Twp =  pRef->GetPoseInverse();

            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_w = Rwp * P3D_ref + twp;
            /*P3D_w.copyTo(mWorldPos);*/
            SetWorldPos(P3D_w);
        }
        else
        {
          return false;
        }
    }
    else{
        cv::Mat P3D_ref = cv::Mat(3,1,5); //in world client

        KeyFrame* pRef = mpMap->GetKeyFrameWithId(pMsg->mpPredKFClientId, pMsg->mpPredKFId);

        if(pRef)
        {
            Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::MP::_mPosPred_type,float>(P3D_ref,pMsg->mPosPred);
        }
        else{
            std::cout<<"There is no pRef for MP???"<<std::endl;
            return false;
        }
        if(!pRef->isBad())
        {
            cv::Mat Twp =  pRef->GetPoseInverse();
            //std::cout<<"pRef:"<<pRef->GetTranslation()<<std::endl;
            //std::cout<<"P3D_ref:"<<P3D_ref<<std::endl;
            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_w = Rwp * P3D_ref + twp;
            SetWorldPos(P3D_w);
            //std::cout<<"P3D_w:"<<P3D_w<<std::endl;
        }
    }

    return true;
}

void MapPoint::ReduceMessage(orb_slam3_ros::MP *pMsgFull, orb_slam3_ros::MPred *pMsgRed)
{
    pMsgRed->mnId = pMsgFull->mnId;
    pMsgRed->mnClientId = pMsgFull->mnClientId;
    pMsgRed->mPosPred = pMsgFull->mPosPred;
    pMsgRed->mPosPar = pMsgFull->mPosPar;
    pMsgRed->mpPredKFId = pMsgFull->mpPredKFId;
    pMsgRed->mpPredKFClientId = pMsgFull->mpPredKFClientId;
    pMsgRed->mpParKFId = pMsgFull->mpParKFId;
    pMsgRed->mpParKFClientId = pMsgFull->mpParKFClientId;
    pMsgRed->mbBad = pMsgFull->mbBad;
}


} //namespace ORB_SLAM
