#include "KeyFrame.h"
#include "Communicator.h"


#include<mutex>

namespace ORB_SLAM3
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0), mbInOutBuffer(false),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),
        mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)), /*mDescriptors(NULL),*/
        /*mBowVec(NULL), mFeatVec(NULL),*/ mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0),
        mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), /*mK(NULL),*/  mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mbIsErasedFromMap(false),
        mHalfBaseline(0), mbCurrentPlaceRecognition(false), mbHasHessian(false), mnMergeCorrectedForKF(0),
        NLeft(0),NRight(0), mnNumberOfOpt(0), mbPoseLock(false)
{

}
KeyFrame::KeyFrame(orb_slam3_ros::KF* msg, Map* pMap, ORBVocabulary* mpVoc, KeyFrameDatabase* pKFDB, Communicator* pComm,
    eSystemState sysState, eSensor Sensor, GeometricCamera* Camera, size_t UniqueId, g2o::Sim3 mg2oS_wcurmap_wclientmap, const IMU::Calib &ImuCalib):
        mnId(msg->mnId), mnClientId(msg->mnClientId), mTimeStamp(msg->dTimestamp), mnGridCols(msg->mnGridCols), mnGridRows(msg->mnGridRows), mbInOutBuffer(false), mbPoseChanged(false),
        mfGridElementWidthInv(msg->mfGridElementWidthInv), mfGridElementHeightInv(msg->mfGridElementHeightInv), mnUniqueId(UniqueId),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(msg->fx), fy(msg->fy), cx(msg->cx), cy(msg->cy), invfx(msg->invfx), invfy(msg->invfy), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(msg->N), mnScaleLevels(msg->mnScaleLevels), mfScaleFactor(msg->mfScaleFactor), mpCamera2(nullptr), mpCamera(Camera),
        mfLogScaleFactor(msg->mfLogScaleFactor), mnMinX(msg->mnMinX), mnMinY(msg->mnMinY), mnMaxX(msg->mnMaxX), mnFrameId(0),
        mnMaxY(msg->mnMaxY),  mbFirstConnection(true), mpParent(NULL),
        mbNotErase(false), mbIsGIBA1(false), mbIsGIBA2(false), mbSendFull(false),
        mbToBeErased(false), mbBad(msg->mbBad), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mbHasHessian(false), mnMergeCorrectedForKF(0),
        /*NLeft(0),NRight(0), */mnNumberOfOpt(0), mpKeyFrameDB(pKFDB), mpORBvocabulary(mpVoc), mSysState(sysState), mpMap(pMap), mSensor(Sensor),
        mbSentOnce(false), mnOriginMapId(msg->mnOriginMapId), mbIsInit(msg->mbIsInit), mbIsVirtualInit(msg->mbIsVirtualInit), mbPoseLock(false), bImu(false)
{
    mspComm.insert(pComm);
    if(mSensor==IMU_MONOCULAR){
        mPrevKF = static_cast<KeyFrame*>(NULL);
        mPrevKFOri = static_cast<KeyFrame*>(NULL);
        mNextKF = static_cast<KeyFrame*>(NULL);
        mpImuPreintegrated = static_cast<IMU::Preintegrated*>(NULL);
        bImu = true;
        Vw = cv::Mat::zeros(3,1,CV_32F);//without initilization mat can be empty
        mImuCalib = ImuCalib;
        mmTcPrev = cv::Mat::eye(4,4,5);//used for erased kf, if this kf is not erased, mmTcPrev should keep eye.
        mmTcPrevOri = cv::Mat::eye(4,4,5);
    }
    //std::cout<<"kf before"<<std::endl;
    if(mSysState == eSystemState::CLIENT)
        mbFromServer = true;
    WriteMembersFromMessage(msg, mg2oS_wcurmap_wclientmap);
    //std::cout<<"kf after"<<std::endl;
    AssignFeaturesToGrid();
    if(mSensor==eSensor::MONOCULAR || mSensor==eSensor::IMU_MONOCULAR){
        mvuRight = vector<float>(N,-1);
        mvDepth = vector<float>(N,-1);
            //Set no stereo fisheye information
        NLeft = -1;
        NRight = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mTlr = cv::Mat(3,4,CV_32F);
        mTrl = cv::Mat(3,4,CV_32F);
    }
    else{
        std::cout<<"At this moment, our sensor can only be MONOCULAR instead of"<<std::endl;
        mvuRight = vector<float>(N,1);//TONGJIANG TODO:might be send from client, DECVIPTORS ALSO DIFFERENT
        mvDepth = vector<float>(N,1);
    }
    if(!mpCamera){
        //std::cout<<"!!!!NO CAMERA MODEL FOR NEW KF."<<std::endl;
        vector<float> vCamCalib{fx,fy,cx,cy};
        mpCamera = new Pinhole(vCamCalib);
    }

}
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, Communicator* pComm, eSystemState sysState, size_t UniqueId):
    mSysState(sysState), mnUniqueId(UniqueId), mbInOutBuffer(false), mbSentOnce(false), mbPoseChanged(false), mnClientId(F.mnClientId),
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
    mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mbIsErasedFromMap(false),
    mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mbHasHessian(false), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch),mTlr(F.mTlr.clone()),
    mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.mTrl), mnNumberOfOpt(0),
    mbSendFull(true), mbIsInit(false),mbIsVirtualInit(false), mbPoseLock(false),
    mbIsGIBA1(false), mbIsGIBA2(false), mbFromServer(false)
{
    //CHANGED
    mspComm.insert(pComm);

    imgLeft = F.imgLeft.clone();
    imgRight = F.imgRight.clone();

    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }



    if(F.mVw.empty())
        Vw = cv::Mat::zeros(3,1,CV_32F);
    else
        Vw = F.mVw.clone();

    mImuBias = F.mImuBias;
    SetPose(F.mTcw);

    mnOriginMapId = pMap->GetId();
}

void KeyFrame::EstablishInitialConnections()
{
    unique_lock<mutex> lock(mMutexFeatures);

    for(int idx=0;idx<mvpMapPoints.size();++idx)
    {
         MapPoint* pMPi = mvpMapPoints[idx];

        if(pMPi)
        {
            pMPi->AddObservation(this, idx);
            pMPi->ComputeDistinctiveDescriptors();
            pMPi->UpdateNormalAndDepth();
        }
        else
        {
                //nullptr -- do not use
        }
    }

}


void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_, bool bLock, bool bLockSend)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT){
        std::cout<<"KF: mbPoseLock "<<std::endl;
        //return;
    }

    {
        unique_lock<mutex> lock(mMutexPose);
        Tcw_.copyTo(Tcw);
        /*if(!mpMap->GetInertialBA2() && mSysState == eSystemState::CLIENT)
            std::cout<<"Setpose Tcw:"<<Tcw<<", of kf:"<<mnId<<std::endl;;*/
      //  if(Tcw.at<float>(0,3)==0 && !mbIsInit)
        //  std::cout<<"Server Tcw"<<Tcw.at<float>(0,3)<<",";
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;
        if (!mImuCalib.Tcb.empty()){
        //    std::cout<<"Ow"<<Ow<<std::endl;
            Owb = Rwc*mImuCalib.Tcb.rowRange(0,3).col(3)+Ow;
        //    std::cout<<"Owb"<<Owb<<std::endl;
        }


        Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));
        cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
        Cw = Twc*center;

        if(mSysState == CLIENT){
            if(IsSent() && !bLockSend){
                mbPoseChanged = true;
                SendMe();
            }
        }
        else if (mSysState == SERVER){
            if(bLock)
                mbPoseLock = true;
        }
    }
}

void KeyFrame::SetVelocity(const cv::Mat &Vw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Vw_.copyTo(Vw);
}


cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return Owb.clone();
}

cv::Mat KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.rowRange(0,3).colRange(0,3)*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
}

cv::Mat KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc*mImuCalib.Tcb;
}

cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}
cv::Mat KeyFrame::GetRotationInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return Vw.clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return vector<KeyFrame*>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<KeyFrame*>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        //cout << "KF WEIGHT n = " << n << endl;
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    /*if(mvpMapPoints[idx]){
        std::cout<<"KF:"<<idx<<"-th feature is associated with MP:"<<mvpMapPoints[idx]->mnId<<std::endl;
        std::cout<<"the new coming MapPoint is:"<<pMP->mnId<<std::endl;*
        return;
    }*/ //TEST BY TONGJIANG
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(this);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex]=static_cast<MapPoint*>(NULL);
    if(rightIndex != -1)
        mvpMapPoints[rightIndex]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent)
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;
    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                continue;
            KFcounter[mit->first]++;

        }
    }
    // This should not happen
    if(KFcounter.empty()){
        std::cout<<"KFcounter empty"<<std::endl;
        return;
    }


    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;//15

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());

    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
        //std::cout<<"KF"<<mnId<<" has "<<mvpOrderedConnectedKeyFrames.size()<<" connected KFs"<<std::endl;
//        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
//        {
//            mpParent = mvpOrderedConnectedKeyFrames.front();
//            mpParent->AddChild(this);
//            mbFirstConnection = false;
//        }

        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            //std::cout<<"Kf:"<<mnId<<" is First Connection"<<std::endl;
            /*if(!mpParent || mpParent->GetParent() != this)
            {
                KeyFrame* pBestParent = static_cast<KeyFrame*>(NULL);
                for(KeyFrame* pKFi : mvpOrderedConnectedKeyFrames)
                {
                    if(pKFi->GetParent() || pKFi->mnId == mpMap->GetInitKFid())
                    {
                        pBestParent = pKFi;
                        break;
                    }
                }
                if(!pBestParent)
                {
                    cout << "It can't be a covisible KF without Parent" << endl << endl;
                    return;
                }
                mpParent = pBestParent;
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }*/  //TONGJIANG: Why this block was out commited
            // cout << "udt.conn.id: " << mnId << endl;

            mpParent = mvpOrderedConnectedKeyFrames.front();//IN CCM parent is not simple as this
            if(mpParent->mnId>mnId)
                cout<<"larger than"<<endl;//until now, this situation not come out
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }

}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
//    if(!mpParent && mpParent != this)
//        mpParent->EraseChild(this);
    if(pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(KeyFrame* pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        std::cout << "Set Erasing KF..." << std::endl;
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
     //std::cout << "Erasing KF..." << std::endl;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            //std::cout << "KF.BADFLAG-> KF 0!!" << std::endl;
            return;
        }
        else if(mbNotErase)
        {
            std::cout << "KF.BADFLAG-> mbNotErase!!" <<mnId<< std::endl;
            mbToBeErased = true;
            return;
        }
        if(!mpParent)
        {
            //cout << "KF.BADFLAG-> There is not parent, but it is not the first KF in the map" << endl;
            //cout << "KF.BADFLAG-> KF: " << mnId << "; first KF: " << mpMap->GetInitKFid() << endl;
        }
    }
    //std::cout << "KF.BADFLAG-> Erasing KF..." << std::endl;

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }
    //std::cout << "KF.BADFLAG-> Connection erased..." << std::endl;

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(this);
            // nDeletedPoints++;
        }
    }
    // cout << "nDeletedPoints: " << nDeletedPoints << endl;
    //std::cout << "KF.BADFLAG-> Observations deleted..." << std::endl;

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);
        //unique_lock<mutex> lock2(mMutexBadFlag);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);
        //std::cout << "KF.BADFLAG-> Initially there are " << sParentCandidates.size() << " candidates" << std::endl;

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }
            //std::cout << "KF.BADFLAG-> Find most similar children" << std::endl;

            if(bContinue)
            {
                if(pC->mnId == pP->mnId)
                {
                    /*cout << "ERROR: The parent and son can't be the same KF. ID: " << pC->mnId << endl;
                    cout << "Current KF: " << mnId << endl;
                    cout << "Parent of the map: " << endl;*/
                }
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }
        //std::cout << "KF.BADFLAG-> Apply change of parent to children" << std::endl;

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }
        //std::cout << "KF.BADFLAG-> Apply change to its parent" << std::endl;

        if(mpParent){
            mpParent->EraseChild(this);
            mTcp = Tcw*mpParent->GetPoseInverse();
        }
        else
        {
            //cout << "Error: KF haven't got a parent, it is imposible reach this code point without him" << endl;
        }
        mbBad = true;
    }

    if(mSysState==CLIENT){
        std::cout << "Sending Erased KF ID..." << std::endl;
        mbIsErasedFromMap = true;
        SendMe();
    }
    else if(mSysState==SERVER){
        //std::cout << "Erasing KF" << std::endl;
    }
    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    //unique_lock<mutex> lock(mMutexConnections);
    //unique_lock<mutex> lock2(mMutexBadFlag);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    //std::cout<<"GetFeaturesInArea start"<<std::endl;
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    /*if(mSysState == eSystemState::SERVER)
        std::cout<<"nMinCellX:"<<nMinCellX<<", nMaxCellX"<<nMaxCellX<<", nMinCellY:"<<nMinCellY<<", nMaxCellY:"<<nMaxCellY<<std::endl;*/
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                /*if(mSysState == eSystemState::SERVER){
                    std::cout<<"kpUn.pt.x:"<<kpUn.pt.x<<",kpUn.pt.y:"<<kpUn.pt.y<<std::endl;
                    std::cout<<"x:"<<x<<",y:"<<y<<std::endl;
                    std::cout<<"distx:"<<distx<<",disty:"<<disty<<std::endl;
                }*/
                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }
    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

cv::Mat KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return (cv::Mat_<float>(3,1) << mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

cv::Mat KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return (cv::Mat_<float>(3,1) << mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void KeyFrame::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the vector
    mvBackupMapPointsId.clear();
    mvBackupMapPointsId.reserve(N);
    for(int i = 0; i < N; ++i)
    {

        if(mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
            mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are positives
            mvBackupMapPointsId.push_back(-1);
    }
    //cout << "KeyFrame: ID from MapPoints stored" << endl;
    // Save the id of each connected KF with it weight
    mBackupConnectedKeyFrameIdWeights.clear();
    for(std::map<KeyFrame*,int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
    {
        if(spKF.find(it->first) != spKF.end())
            mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }
    //cout << "KeyFrame: ID from connected KFs stored" << endl;
    // Save the parent id
    mBackupParentId = -1;
    if(mpParent && spKF.find(mpParent) != spKF.end())
        mBackupParentId = mpParent->mnId;
    //cout << "KeyFrame: ID from Parent KF stored" << endl;
    // Save the id of the childrens KF
    mvBackupChildrensId.clear();
    mvBackupChildrensId.reserve(mspChildrens.size());
    for(KeyFrame* pKFi : mspChildrens)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupChildrensId.push_back(pKFi->mnId);
    }
    //cout << "KeyFrame: ID from Children KFs stored" << endl;
    // Save the id of the loop edge KF
    mvBackupLoopEdgesId.clear();
    mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
    for(KeyFrame* pKFi : mspLoopEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }
    //cout << "KeyFrame: ID from Loop KFs stored" << endl;
    // Save the id of the merge edge KF
    mvBackupMergeEdgesId.clear();
    mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
    for(KeyFrame* pKFi : mspMergeEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }
    //cout << "KeyFrame: ID from Merge KFs stored" << endl;

    //Camera data
    mnBackupIdCamera = -1;
    if(mpCamera && spCam.find(mpCamera) != spCam.end())
        mnBackupIdCamera = mpCamera->GetId();
    //cout << "KeyFrame: ID from Camera1 stored; " << mnBackupIdCamera << endl;

    mnBackupIdCamera2 = -1;
    if(mpCamera2 && spCam.find(mpCamera2) != spCam.end())
        mnBackupIdCamera2 = mpCamera2->GetId();
    //cout << "KeyFrame: ID from Camera2 stored; " << mnBackupIdCamera2 << endl;

    //Inertial data
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;
    //cout << "KeyFrame: ID from Prev KF stored" << endl;
    mBackupNextKFId = -1;
    if(mNextKF && spKF.find(mNextKF) != spKF.end())
        mBackupNextKFId = mNextKF->mnId;
    //cout << "KeyFrame: ID from NextKF stored" << endl;
    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
    //cout << "KeyFrame: Imu Preintegrated stored" << endl;
}

void KeyFrame::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId){
    // Rebuild the empty variables

    // Pose
    SetPose(Tcw);

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mvpMapPoints.clear();
    mvpMapPoints.resize(N);
    for(int i=0; i<N; ++i)
    {
        if(mvBackupMapPointsId[i] != -1)
            mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
        else
            mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
    }

    // Conected KeyFrames with him weight
    mConnectedKeyFrameWeights.clear();
    for(map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end();
        it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        mConnectedKeyFrameWeights[pKFi] = it->second;
    }

    // Restore parent KeyFrame
    if(mBackupParentId>=0)
        mpParent = mpKFid[mBackupParentId];

    // KeyFrame childrens
    mspChildrens.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it!=end; ++it)
    {
        mspChildrens.insert(mpKFid[*it]);
    }

    // Loop edge KeyFrame
    mspLoopEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
    {
        mspLoopEdges.insert(mpKFid[*it]);
    }

    // Merge edge KeyFrame
    mspMergeEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
    {
        mspMergeEdges.insert(mpKFid[*it]);
    }

    //Camera data
    if(mnBackupIdCamera >= 0)
    {
        mpCamera = mpCamId[mnBackupIdCamera];
    }
    if(mnBackupIdCamera2 >= 0)
    {
        mpCamera2 = mpCamId[mnBackupIdCamera2];
    }

    //Inertial data
    if(mBackupPrevKFId != -1)
    {
        mPrevKF = mpKFid[mBackupPrevKFId];
    }
    if(mBackupNextKFId != -1)
    {
        mNextKF = mpKFid[mBackupNextKFId];
    }
    mpImuPreintegrated = &mBackupImuPreintegrated;


    // Remove all backup container
    mvBackupMapPointsId.clear();
    mBackupConnectedKeyFrameIdWeights.clear();
    mvBackupChildrensId.clear();
    mvBackupLoopEdgesId.clear();

    UpdateBestCovisibles();

    //ComputeSceneMedianDepth();
}

bool KeyFrame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);

    // 3D in camera coordinates
    cv::Mat Pc = Rcw*P+tcw;
    float &PcX = Pc.at<float>(0);
    float &PcY= Pc.at<float>(1);
    float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    // 3D in camera coordinates
    cv::Mat Pc = Rcw*P+tcw;
    float &PcX = Pc.at<float>(0);
    float &PcY= Pc.at<float>(1);
    float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

cv::Mat KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rrw = Rrl * Rlw;

    cv::Mat tlw = Tcw.rowRange(0,3).col(3).clone();
    cv::Mat trl = - Rrl * mTlr.rowRange(0,3).col(3);

    cv::Mat trw = Rrl * tlw + trl;

    cv::Mat Trw;
    cv::hconcat(Rrw,trw,Trw);

    return Trw.clone();
}

cv::Mat KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rwr = (Rrl * Rlw).t();

    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = GetCameraCenter();

    cv::Mat twr = Rwl * tlr + twl;

    cv::Mat Twr;
    cv::hconcat(Rwr,twr,Twr);

    return Twr.clone();
}

cv::Mat KeyFrame::GetRightPoseInverseH() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rwr = (Rrl * Rlw).t();

    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = Ow.clone();

    cv::Mat twr = Rwl * tlr + twl;

    cv::Mat Twr;
    cv::hconcat(Rwr,twr,Twr);
    cv::Mat h(1,4,CV_32F,cv::Scalar(0.0f)); h.at<float>(3) = 1.0f;
    cv::vconcat(Twr,h,Twr);

    return Twr.clone();
}

cv::Mat KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = Ow.clone();

    cv::Mat twr = Rwl * tlr + twl;

    return twr.clone();
}

cv::Mat KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rrw = Rrl * Rlw;

    return Rrw.clone();

}

cv::Mat KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlw = Tcw.rowRange(0,3).col(3).clone();
    cv::Mat trl = - Rrl * mTlr.rowRange(0,3).col(3);

    cv::Mat trw = Rrl * tlw + trl;

    return trw.clone();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}


//FOR COMMUNICATION
orb_slam3_ros::KF KeyFrame::GetKFMessageClient(){ //NOT USED
    orb_slam3_ros::KF Msg;
    Msg.mnId = mnId;
    Msg.mnClientId = mnClientId;
    Msg.dTimestamp = mTimeStamp;
    Msg.mnGridCols = mnGridCols;
    Msg.mnGridRows = mnGridRows;
    Msg.mfGridElementWidthInv = mfGridElementWidthInv;
    Msg.mfGridElementHeightInv = mfGridElementHeightInv;
    Msg.fx = fx;
    Msg.fy = fy;
    Msg.cx = cx;
    Msg.cy = cy;
    Msg.invfx = invfx;
    Msg.invfy = invfy;
    Msg.N = static_cast<int16_t>(N);
    Msg.mnScaleLevels = static_cast<int8_t>(mnScaleLevels);
    Msg.mfScaleFactor = mfScaleFactor;
    Msg.mfLogScaleFactor = mfLogScaleFactor;
    for(int idx=0;idx<mvScaleFactors.size();++idx) Msg.mvScaleFactors[idx]=mvScaleFactors[idx];
    for(int idx=0;idx<mvLevelSigma2.size();++idx) Msg.mvLevelSigma2[idx]=mvLevelSigma2[idx];
    for(int idx=0;idx<mvInvLevelSigma2.size();++idx) Msg.mvInvLevelSigma2[idx]=mvInvLevelSigma2[idx];
    Msg.mnMinX = static_cast<int16_t>(mnMinX);
    Msg.mnMinY = static_cast<int16_t>(mnMinY);
    Msg.mnMaxX = static_cast<int16_t>(mnMaxX);
    Msg.mnMaxY = static_cast<int16_t>(mnMaxY);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mK_type,float>(mK,Msg.mK);

    for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

    for(int idx=0;idx<mDescriptors.rows;++idx)
    {
        orb_slam3_ros::Descriptor MsgDesc;
        for(int idy=0;idy<mDescriptors.cols;++idy)
        {
            MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
        }
        Msg.mDescriptors.push_back(MsgDesc);
    }
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcw_type,float>(Tcw,Msg.mTcw);

    for(int idx=0;idx<mvpMapPoints.size();++idx)
    {
        if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
        {
            Msg.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mnId));
            Msg.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mnClientId));
            Msg.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
        }
    }
    Msg.bSentOnce =  mbSentOnce;
    mbSentOnce = true;
    Msg.mbPoseChanged = mbPoseChanged;
    mbPoseChanged=false;
    return Msg;
}

void KeyFrame::SendMe()
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SendMe(): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

    if(IsInOutBuffer()&&mbIsErasedFromMap){
        cout<<"IsInOutBuffer"<<endl;
        for(set<Communicator*>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            Communicator* pComm = *sit;
            if(pComm->mnClientId==mnClientId){
                cout<<"EraseKfinBuffer"<<endl;
                pComm->ErasedKfFromBuffer(this);
                return;
            }
        }
    }

    if(IsSent() && !IsInOutBuffer())
    {
        for(set<Communicator*>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            Communicator* pComm = *sit;
            if(mbIsErasedFromMap && pComm->mnClientId==mnClientId){
                cout<<"PassErasedKfIdtoComm"<<endl;
                pComm->PassErasedKfIdtoComm(mnId, mpMap->GetInertialBA1());
                return;
            }
            if(pComm==nullptr){
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SendMe(): no Comm" << endl;
                throw infrastructure_ex();
            }
            else{
                //std::cout<<"KF: before pass kf to comm"<<std::endl;
                pComm->PassKftoComm(this, mpMap->GetInertialBA1());
                //std::cout<<"KF: after pass kf to comm"<<std::endl;
                //pComm->PassKftoComm(this, mpMap->isImuInitialized());
            }

        }
    }
}

void KeyFrame::ConvertToMessage(orb_slam3_ros::Map &msgMap)
{
    unique_lock<mutex> lockOut(mMutexOut);

    if(mbSendFull)
    {
        orb_slam3_ros::KF Msg;

        unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
        unique_lock<mutex> lock2(mMutexConnections,defer_lock);
        unique_lock<mutex> lock3(mMutexPose,defer_lock);

        lock(lock1,lock2,lock3);
    //    std::cout<<mnId<<std::endl;
        Msg.mnId = static_cast<uint16_t>(mnId);
    //    std::cout<<Msg.mnId<<std::endl;
        Msg.mbIsGIBA1 = this->GetInertialBA1();
        Msg.mbIsGIBA2 = this->GetInertialBA2();
        Msg.mnOriginMapId = static_cast<uint16_t>(mnOriginMapId);
        Msg.mnClientId = static_cast<uint8_t>(mnClientId);
        Msg.mbIsInit = mbIsInit;
        Msg.mbIsVirtualInit = mbIsVirtualInit;
        Msg.dTimestamp = mTimeStamp;
        Msg.mnGridCols = mnGridCols;
        Msg.mnGridRows = mnGridRows;
        Msg.mfGridElementWidthInv = mfGridElementWidthInv;
        Msg.mfGridElementHeightInv = mfGridElementHeightInv;
        Msg.fx = fx;
        Msg.fy = fy;
        Msg.cx = cx;
        Msg.cy = cy;
        Msg.invfx = invfx;
        Msg.invfy = invfy;
        Msg.N = static_cast<int16_t>(N);
        Msg.mnScaleLevels = static_cast<int8_t>(mnScaleLevels);
        Msg.mfScaleFactor = mfScaleFactor;
        Msg.mfLogScaleFactor = mfLogScaleFactor;
        for(int idx=0;idx<mvScaleFactors.size();++idx) Msg.mvScaleFactors[idx]=mvScaleFactors[idx];
        for(int idx=0;idx<mvLevelSigma2.size();++idx) Msg.mvLevelSigma2[idx]=mvLevelSigma2[idx];
        for(int idx=0;idx<mvInvLevelSigma2.size();++idx) Msg.mvInvLevelSigma2[idx]=mvInvLevelSigma2[idx];
        Msg.mnMinX = static_cast<int16_t>(mnMinX);
        Msg.mnMinY = static_cast<int16_t>(mnMinY);
        Msg.mnMaxX = static_cast<int16_t>(mnMaxX);
        Msg.mnMaxY = static_cast<int16_t>(mnMaxY);
        Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mK_type,float>(mK,Msg.mK);

        for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

        for(int idx=0;idx<mDescriptors.rows;++idx)
        {
            orb_slam3_ros::Descriptor MsgDesc;
            for(int idy=0;idy<mDescriptors.cols;++idy)
            {
                MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
            }
            Msg.mDescriptors.push_back(MsgDesc);
        }
        Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcw_type,float>(Tcw,Msg.mTcw);
        //std::cout<<"01 tcw msg:"<<Msg.mTcw[0]<<","<<Msg.mTcw[3]<<std::endl;
        if(Msg.mTcw[3]==0 && !mbIsInit)
          std::cout<<"KF:01"<<std::endl;
        //USE RELATIVE POSE INSTEAD OF ABSOLUTE
        if(mbIsInit){
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcpred_type,float>(Tcw,Msg.mTcpred);
            if(mpImuPreintegrated){ //but init frame acutually doesn't has it,right?
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mVcp_type,float>(Vw,Msg.mVcp);
            /*    Msg.mVcp[0] = Vw[0];
                Msg.mVcp[1] = Vw[1];
                Msg.mVcp[2] = Vw[2];*/
                Msg.mGb[0] = mImuBias.bwx;
                Msg.mGb[1] = mImuBias.bwy;
                Msg.mGb[2] = mImuBias.bwz;
                Msg.mAb[0] = mImuBias.bax;
                Msg.mAb[1] = mImuBias.bay;
                Msg.mAb[2] = mImuBias.baz;
            }
        }
        else
        {
            //pose relative to predecessor
            KeyFrame* pPred = mpMap->GetPredecessor(this);
            cv::Mat mTcpred = Tcw*pPred->GetPoseInverse();
            Msg.mpPred_KfId = static_cast<uint16_t>(pPred->mnId);
            Msg.mpPred_KfClientId = static_cast<uint8_t>(pPred->mnClientId);
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcpred_type,float>(mTcpred,Msg.mTcpred);
            if(pPred){
                KeyFrame* pPredPred = mpMap->GetPredecessor(pPred);
                cv::Mat mTcpredpred = Tcw*pPredPred->GetPoseInverse();
                Msg.mpPredPred_KfId = static_cast<uint16_t>(pPredPred->mnId);
                Msg.mpPredPred_KfClientId = static_cast<uint8_t>(pPredPred->mnClientId);
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcpredpred_type,float>(mTcpredpred,Msg.mTcpredpred);
            }
            //pose relative to parent's parent
            if(!mpParent)
            {
                cout << "Fetal : no parent" << endl;
                throw infrastructure_ex();
            }
            else
            {
                /*KeyFrame* pParPar = mpParent->GetParent();
                if(!pParPar)
                    pParPar = mpParent; //if mpParents has no parent, use mpParent
                else if(pParPar->mId == this->mId)
                {
                        pParPar = mpParent; //if mpParents has no parent, use mpParent
                }*/

                cv::Mat mTcpar = Tcw*mpParent->GetPoseInverse();
                Msg.mpPar_KfId = static_cast<uint16_t>(mpParent->mnId);
                Msg.mpPar_KfClientId = static_cast<uint8_t>(mpParent->mnClientId);
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcpar_type,float>(mTcpar,Msg.mTcpar);

            }

            //FOR IMU
            if(mpImuPreintegrated && mPrevKF){
                Msg.mpPrevKfId = static_cast<uint16_t>(mPrevKF->mnId);
                Msg.mpPrevKfClientId = static_cast<uint8_t>(mPrevKF->mnClientId);
                //std::cout<<Vw.cols<<std::endl;
                //std::cout<<(mPrevKF->GetRotationInverse()).rows<<std::endl;
                cv::Mat mVcp = Vw.t() * mPrevKF->GetRotationInverse(); //TODO: TO BE CHECKED
                //std::cout<<mVcp.size()<<std::endl;
                cv::Mat mTcprev = Tcw * mPrevKF->GetPoseInverse();
                //std::cout<<mTcprev.size()<<std::endl;
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mVcp_type,float>(mVcp,Msg.mVcp);
                //std::cout<<"velocity:"<<Msg.mVcp[0]<<","<<Msg.mVcp[1]<<","<<Msg.mVcp[2]<<std::endl;
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcprev_type,float>(mTcprev,Msg.mTcprev);
                Msg.mGb[0] = mImuBias.bwx;
                Msg.mGb[1] = mImuBias.bwy;
                Msg.mGb[2] = mImuBias.bwz;
                Msg.mAb[0] = mImuBias.bax;
                Msg.mAb[1] = mImuBias.bay;
                Msg.mAb[2] = mImuBias.baz;
                ConvertToImuMessage(Msg.mImuPreintegrated);
            }
        }

        for(int idx=0;idx<mvpMapPoints.size();++idx)
        {
            if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
            {
                Msg.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mnId));
                Msg.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mnClientId));
                Msg.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
            }
        }
        Msg.bSentOnce =  mbSentOnce;
        mbSentOnce = true;
        mbPoseChanged=false;
        mbSendFull = false;

        msgMap.Keyframes.push_back(Msg);
    }
    else
    {
        orb_slam3_ros::KFred Msg;

        unique_lock<mutex> lockPose(mMutexPose);

        if(mbPoseChanged)
        {
            mbPoseChanged = false;

            Msg.mbBad = false; //necessary in case that KF is in out buffer when trimmed from Map
            Msg.mnId = static_cast<uint16_t>(mnId);
            Msg.mnClientId = static_cast<uint8_t>(mnClientId);
            Msg.mnOriginMapId = static_cast<uint16_t>(mnOriginMapId);
            Msg.mnUniqueId = static_cast<uint32_t>(mnUniqueId);
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcw_type,float>(Tcw,Msg.mTcw);
            if(mbIsInit){
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcpred_type,float>(Tcw,Msg.mTcpred);
                //CHANGED BY TONGJIANG
            /*    Msg.mpPred_KfId = static_cast<uint16_t>(mnId);
                Msg.mpPred_KfClientId = static_cast<uint8_t>(mnClientId);
                Msg.mpPar_KfId = static_cast<uint16_t>(mnId);
                Msg.mpPar_KfClientId = static_cast<uint8_t>(mnClientId);*/
                if(mpImuPreintegrated){
                    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mVcp_type,float>(Vw,Msg.mVcp);
                    Msg.mGb[0] = mImuBias.bwx;
                    Msg.mGb[1] = mImuBias.bwy;
                    Msg.mGb[2] = mImuBias.bwz;
                    Msg.mAb[0] = mImuBias.bax;
                    Msg.mAb[1] = mImuBias.bay;
                    Msg.mAb[2] = mImuBias.baz;
                }
            }
            else
            {
                //pose relative to predecessor
                KeyFrame* pPred = mpMap->GetPredecessor(this);
                cv::Mat mTcpred = Tcw*pPred->GetPoseInverse();
                Msg.mpPred_KfId = static_cast<uint16_t>(pPred->mnId);
                Msg.mpPred_KfClientId = static_cast<uint8_t>(pPred->mnClientId);
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcpred_type,float>(mTcpred,Msg.mTcpred);
                if(pPred){
                    KeyFrame* pPredPred = mpMap->GetPredecessor(pPred);
                    cv::Mat mTcpredpred = Tcw*pPredPred->GetPoseInverse();
                    Msg.mpPredPred_KfId = static_cast<uint16_t>(pPredPred->mnId);
                    Msg.mpPredPred_KfClientId = static_cast<uint8_t>(pPredPred->mnClientId);
                    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcpredpred_type,float>(mTcpredpred,Msg.mTcpredpred);
                }
                //pose relative to parent's parent
                if(!mpParent)
                {
                    cout << "Fetal : no parent" << endl;
                    throw infrastructure_ex();
                }
                else
                {
                /*KeyFrame* pParPar = mpParent->GetParent();
                if(!pParPar)
                    pParPar = mpParent; //if mpParents has no parent, use mpParent
                else if(pParPar->mId == this->mId)
                {
                        pParPar = mpParent; //if mpParents has no parent, use mpParent
                }*/

                    cv::Mat mTcpar = Tcw*mpParent->GetPoseInverse();
                    Msg.mpPar_KfId = static_cast<uint16_t>(mpParent->mnId);
                    Msg.mpPar_KfClientId = static_cast<uint8_t>(mpParent->mnClientId);
                    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcpar_type,float>(mTcpar,Msg.mTcpar);
                }
                //FOR IMU
                if(mpImuPreintegrated && mPrevKF){
                    Msg.mpPrevKfId = static_cast<uint16_t>(mPrevKF->mnId);
                    Msg.mpPrevKfClientId = static_cast<uint8_t>(mPrevKF->mnClientId);
                    cv::Mat mVcp = Vw.t() * mPrevKF->GetRotationInverse(); //TODO: TO BE CHECKED
                    cv::Mat mTcprev = Tcw * mPrevKF->GetPoseInverse();
                    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mVcp_type,float>(mVcp,Msg.mVcp);
                    //std::cout<<"velocity:"<<Msg.mVcp[0]<<","<<Msg.mVcp[1]<<","<<Msg.mVcp[2]<<std::endl;
                    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcprev_type,float>(mTcprev,Msg.mTcprev);
                    Msg.mGb[0] = mImuBias.bwx;
                    Msg.mGb[1] = mImuBias.bwy;
                    Msg.mGb[2] = mImuBias.bwz;
                    Msg.mAb[0] = mImuBias.bax;
                    Msg.mAb[1] = mImuBias.bay;
                    Msg.mAb[2] = mImuBias.baz;
                    ConvertToImuMessage(Msg.mImuPreintegrated);
                }
            }
            /*for(int idx=0;idx<mvpMapPoints.size();++idx)
            {
                if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
                {
                    Msg.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mnId));
                    Msg.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mnClientId));
                    Msg.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
                }
            }*/

            msgMap.KFUpdates.push_back(Msg);
        }
        else
        {
                //pose has not changed - do nothing
        }
    }
}

void KeyFrame::ConvertToMessageServer(orb_slam3_ros::Map &msgMap, KeyFrame* pKFref, uint8_t nClientIdComm)
{
    unique_lock<mutex> lockOut(mMutexOut);
    if(!pKFref)
        return;
    if(nClientIdComm==mnClientId || (nClientIdComm!=mnClientId && mbSentOnce)){
        /*if(nClientIdComm!=mnClientId && mbSentOnce)
            std::cout<<"!!!!!!!!!!sentOnce so only reduced info"<<std::endl;*/
        orb_slam3_ros::KFred pMsgRed;
        pMsgRed.mbBad = false; //necessary in case that KF is in out buffer when trimmed from Map
        pMsgRed.mbPoseLock = mbPoseLock;
        pMsgRed.mnId = static_cast<uint16_t>(mnId);
        pMsgRed.mnClientId = static_cast<uint8_t>(mnClientId);
        pMsgRed.mnOriginMapId = static_cast<uint16_t>(mnOriginMapId);
        pMsgRed.mnUniqueId = static_cast<uint32_t>(mnUniqueId);
        Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcw_type,float>(Tcw,pMsgRed.mTcw);
        if(mbIsInit){
            return; // we do not send the init kf
        }
        else
        {
            //pose relative to ref kf
            cv::Mat mTcref = Tcw * pKFref->GetPoseInverse();
            pMsgRed.mpPred_KfId = static_cast<uint16_t>(pKFref->mnId);
            pMsgRed.mpPred_KfClientId = static_cast<uint8_t>(pKFref->mnClientId);
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KFred::_mTcpred_type,float>(mTcref,pMsgRed.mTcpred);
        }

        for(int idx=0;idx<mvpMapPoints.size();++idx)
        {
            if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
            {
                pMsgRed.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mnId));
                pMsgRed.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mnClientId));
                pMsgRed.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
            }
        } //TODO MAYBE ONLY FOR ANOTHER CLIENT?

        msgMap.KFUpdates.push_back(pMsgRed);
    }
    else{
        std::cout<<"send full kf msg"<<std::endl;//TODO
        mbSendFull = true;
        orb_slam3_ros::KF Msg;
        {
            unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
            unique_lock<mutex> lock2(mMutexConnections,defer_lock);
            unique_lock<mutex> lock3(mMutexPose,defer_lock);

            lock(lock1,lock2,lock3);

            //USE RELATIVE POSE INSTEAD OF ABSOLUTE
            if(mbIsInit){
                return;
            }
            else
            {
                //std::cout<<"Tcw:"<<Tcw<<std::endl;
                //std::cout<<"Trefw:"<<pKFref->GetPose()<<std::endl;
                cv::Mat mTcref = Tcw * pKFref->GetPoseInverse();
                //std::cout<<"mTcref:"<<mTcref<<std::endl;
                Msg.mpPred_KfId = static_cast<uint16_t>(pKFref->mnId);
                Msg.mpPred_KfClientId = static_cast<uint8_t>(pKFref->mnClientId);
                Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcpred_type,float>(mTcref,Msg.mTcpred);
            }

            for(int idx=0;idx<mvpMapPoints.size();++idx)
            {
                if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
                {
                    Msg.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mnId));
                    Msg.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mnClientId));
                    Msg.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
                }
            }
            //Msg.bSentOnce =  mbSentOnce;
            Msg.mnId = static_cast<uint16_t>(mnId);
            Msg.mnUniqueId = static_cast<uint32_t>(mnUniqueId);
            Msg.mbIsGIBA1 = this->GetInertialBA1();
            Msg.mbIsGIBA2 = this->GetInertialBA2();
            Msg.mnOriginMapId = static_cast<uint16_t>(mnOriginMapId);
            Msg.mnClientId = static_cast<uint8_t>(mnClientId);
            Msg.mbIsInit = mbIsInit;
            Msg.mbIsVirtualInit = mbIsVirtualInit;
            Msg.dTimestamp = mTimeStamp;
            Msg.mnGridCols = mnGridCols;
            Msg.mnGridRows = mnGridRows;
            Msg.mfGridElementWidthInv = mfGridElementWidthInv;
            Msg.mfGridElementHeightInv = mfGridElementHeightInv;
            Msg.fx = fx;
            Msg.fy = fy;
            Msg.cx = cx;
            Msg.cy = cy;
            Msg.invfx = invfx;
            Msg.invfy = invfy;
            Msg.N = static_cast<int16_t>(N);
            Msg.mnScaleLevels = static_cast<int8_t>(mnScaleLevels);
            Msg.mfScaleFactor = mfScaleFactor;
            Msg.mfLogScaleFactor = mfLogScaleFactor;
            for(int idx=0;idx<mvScaleFactors.size();++idx) Msg.mvScaleFactors[idx]=mvScaleFactors[idx];
            for(int idx=0;idx<mvLevelSigma2.size();++idx) Msg.mvLevelSigma2[idx]=mvLevelSigma2[idx];
            for(int idx=0;idx<mvInvLevelSigma2.size();++idx) Msg.mvInvLevelSigma2[idx]=mvInvLevelSigma2[idx];
            Msg.mnMinX = static_cast<int16_t>(mnMinX);
            Msg.mnMinY = static_cast<int16_t>(mnMinY);
            Msg.mnMaxX = static_cast<int16_t>(mnMaxX);
            Msg.mnMaxY = static_cast<int16_t>(mnMaxY);
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mK_type,float>(mK,Msg.mK);

            for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

            for(int idx=0;idx<mDescriptors.rows;++idx)
            {
                orb_slam3_ros::Descriptor MsgDesc;
                for(int idy=0;idy<mDescriptors.cols;++idy)
                {
                    MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
                }
                Msg.mDescriptors.push_back(MsgDesc);
            }
            Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::KF::_mTcw_type,float>(Tcw,Msg.mTcw);
        }
        mbSentOnce = true;
        std::cout<<"finish send full kf msg"<<std::endl;
        msgMap.Keyframes.push_back(Msg);
    }
}

void KeyFrame::ConvertToImuMessage(orb_slam3_ros::PreintegratedIMU &msg_imu){
    msg_imu.dT = mpImuPreintegrated->dT;
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_C_type,float>(mpImuPreintegrated->C,msg_imu.C);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_Nga_type,float>(mpImuPreintegrated->Nga,msg_imu.Nga);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_NgaWalk_type,float>(mpImuPreintegrated->NgaWalk,msg_imu.NgaWalk);
    //Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_b_type,float>(mpImuPreintegrated->b,msg_imu.b);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_dR_type,float>(mpImuPreintegrated->dR,msg_imu.dR);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_dV_type,float>(mpImuPreintegrated->dV,msg_imu.dV);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_dP_type,float>(mpImuPreintegrated->dP,msg_imu.dP);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_JRg_type,float>(mpImuPreintegrated->JRg,msg_imu.JRg);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_JVg_type,float>(mpImuPreintegrated->JVg,msg_imu.JVg);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_JVa_type,float>(mpImuPreintegrated->JVa,msg_imu.JVa);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_JPg_type,float>(mpImuPreintegrated->JPg,msg_imu.JPg);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_JPa_type,float>(mpImuPreintegrated->JPa,msg_imu.JPa);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_avgA_type,float>(mpImuPreintegrated->avgA,msg_imu.avgA);
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::PreintegratedIMU::_avgW_type,float>(mpImuPreintegrated->avgW,msg_imu.avgW);
    msg_imu.b[0] = mpImuPreintegrated->b.bax;
    msg_imu.b[1] = mpImuPreintegrated->b.bay;
    msg_imu.b[2] = mpImuPreintegrated->b.baz;
    msg_imu.b[3] = mpImuPreintegrated->b.bwx;
    msg_imu.b[4] = mpImuPreintegrated->b.bwy;
    msg_imu.b[5] = mpImuPreintegrated->b.bwz;
}


void KeyFrame::WriteMembersFromMessage(orb_slam3_ros::KF* msg, g2o::Sim3 mg2oS_wcurmap_wclientmap){
    if(mSensor==IMU_MONOCULAR){
        mbIsGIBA1 = msg->mbIsGIBA1;
        mbIsGIBA2 = msg->mbIsGIBA2;
    }
    for(int idx=0;idx<msg->mvScaleFactors.size();++idx) mvScaleFactors.push_back(msg->mvScaleFactors[idx]);
    for(int idx=0;idx<msg->mvLevelSigma2.size();++idx) mvLevelSigma2.push_back(msg->mvLevelSigma2[idx]);
    for(int idx=0;idx<msg->mvInvLevelSigma2.size();++idx) mvInvLevelSigma2.push_back(msg->mvInvLevelSigma2[idx]);
    mK = cv::Mat::eye(3,3,CV_32F);
    Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mK_type, float>(mK, msg->mK);
    mvKeysUn.resize(msg->mvKeysUn.size());
    for(int idx=0;idx<msg->mvKeysUn.size();++idx)
        mvKeysUn[idx] = Converter::fromCvKeyPointMsg(msg->mvKeysUn[idx]);
    orb_slam3_ros::Descriptor TempDesc = msg->mDescriptors[0];
    int iBoundY = static_cast<int>(TempDesc.mDescriptor.size());
    int iBoundX = static_cast<int>(msg->mDescriptors.size());
    mDescriptors = cv::Mat(iBoundX,iBoundY,0);
    for(int idx=0;idx<msg->mDescriptors.size();++idx)
    {
        orb_slam3_ros::Descriptor MsgDesc = msg->mDescriptors[idx];
        for(int idy=0;idy<iBoundY;++idy){
            mDescriptors.at<uint8_t>(idx, idy) = MsgDesc.mDescriptor[idy];
        }
    }
    if(!mBowVec.empty() || !mFeatVec.empty())
    {
        std::cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " !mBowVec.empty() || !mFeatVec.empty()" << std::endl;
    }

    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    //find mappoints
    mvpMapPoints.resize(N,nullptr);
    int numMP = 0;
    int numNotFound = 0;
    for(int idx=0;idx<msg->mvpMapPoints_Ids.size();++idx)
    {
        size_t FeatId = msg->mvpMapPoints_VectId[idx];
        //MapPoint* pMP = mpMap->GetMapPointWithId(mnClientId,msg->mvpMapPoints_Ids[idx]); //TODO
        MapPoint* pMP = mpMap->GetMapPointWithId(
            msg->mvpMapPoints_ClientIds[idx], msg->mvpMapPoints_Ids[idx]
        );
        if(pMP)
        {
            mvpMapPoints[FeatId] = pMP;
            numMP++;
        }
        else
        {
            numNotFound++;
            //std::cout<<"map point not found!"<<std::endl;
            //if MP not in Map, we ignore it. Might be deleted, or comes in later and is then (hopefully) added to this KF
            //TONGJIANG:really hopefully....but actually not....
        }
    }
    std::cout<<"number of not found mp:"<<numNotFound<<", found:"<<numMP<<std::endl;
    if(numMP==0) //its not a big problem, since server is also like this at the beginning.
        std::cout<<"WARNING:no mp found!!!"<<std::endl;

  //POSE
    if(mSysState == eSystemState::SERVER)
    {
      //  unique_lock<mutex> lock2(mMutexBadFlag);
        // SERVER
     /*   cv::Mat T_SC = cv::Mat(4,4,5);
        Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mT_SC_type,float>(T_SC,msg->mT_SC);
        mT_SC = Converter::toMatrix4d(T_SC);*/
        //std::cout<<"tcw msg:"<<msg->mTcw[0]<<","<<msg->mTcw[3]<<std::endl;
        if(!msg->mbIsInit)
        {
            bool bSetPose = SetPoseFromMessage(msg,mg2oS_wcurmap_wclientmap);

            if(!bSetPose)
            {
                mvpMapPoints.clear();
                mbBad = true;
                std::cout<<"bSetPose BAD"<<std::endl;
                return;
            }
            if(Tcw.at<float>(0,3)==0 && Tcw.at<float>(0,0)==0){
              //  std::cout<<"Client:"<<unsigned(mnClientId)<<", KF:"<<mnId<<"is Bad in server"<<std::endl;
                mbBad = true;
            }
        }
        else
        {
            if(!msg->mbIsVirtualInit){
               //first KF has no parent
                cv::Mat tempTcw = cv::Mat(4,4,5);
                Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcpred_type,float>(tempTcw,msg->mTcpred);
                float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
                tempTcw.at<float>(0,3) /=(1./s);
                tempTcw.at<float>(1,3) /=(1./s);
                tempTcw.at<float>(2,3) /=(1./s);
                SetPose(tempTcw);
                if((Tcw.at<float>(0,3)==0 && Tcw.at<float>(0,0)==0) || mbBad){
                    std::cout<<"Client:"<<unsigned(mnClientId)<<", init KF:"<<mnId<<"is Bad in server"<<std::endl;
                    std::cout<<"init mually set pose"<<std::endl;
                    tempTcw.at<float>(0,0)=1;
                    tempTcw.at<float>(1,1)=1;
                    tempTcw.at<float>(2,2)=1;
                    SetPose(tempTcw);
                }
            }
            else{  //"kf in new map is in, but there is no new map" so we need a virtual init kf
                std::cout<<"Creating a virtual initial KF"<<std::endl;
                if(mbIsInit)
                    std::cout<<"KF"<<mnId<<"is init"<<std::endl;
                cv::Mat tempTcw = cv::Mat(4,4,5);
                Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcw_type,float>(tempTcw,msg->mTcw);
                //Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcpred_type,float>(tempTcw,msg->mTcpred);
                std::cout<<"virtual pose:"<<tempTcw.at<float>(0,0)<<","<<tempTcw.at<float>(0,3)<<std::endl;
                std::cout<<"virtual tcw msg:"<<msg->mTcw[0]<<","<<msg->mTcw[3]<<std::endl;
                /*float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
                tempTcw.at<float>(0,3) /=(1./s);
                tempTcw.at<float>(1,3) /=(1./s);
                tempTcw.at<float>(2,3) /=(1./s);*/

                SetPose(tempTcw);  //TODO TONGJIANG at the moment we use its absolute pose cause we think its reference coordinate is (0,0,0),
                if((Tcw.at<float>(0,3)==0 && Tcw.at<float>(0,0)==0) || mbBad){
                    std::cout<<"Client:"<<unsigned(mnClientId)<<", virtual init KF:"<<mnId<<"is Bad in server"<<std::endl;
                    std::cout<<"virtua init mually set pose"<<std::endl;
                    tempTcw.at<float>(0,0)=1;
                    tempTcw.at<float>(1,1)=1;
                    tempTcw.at<float>(2,2)=1;
                    SetPose(tempTcw);
                }
            }

        }
    }
    else{
        //std::cout<<"CLIENT IS WRITING FROM MESSAGE."<<std::endl;
        bool bSetPose = SetPoseFromMessage(msg,mg2oS_wcurmap_wclientmap);
        if(!bSetPose)
        {
            mvpMapPoints.clear();
            mbBad = true;
            std::cout<<"bSetPose BAD"<<std::endl;
            return;
        }
    }

}

void KeyFrame::UpdateFromMessage(orb_slam3_ros::KFred *msg, g2o::Sim3 mg2oS_wcurmap_wclientmap, bool bUpdateMPs)
{
   // mbOmitSending = true; //TODO....
    unique_lock<mutex> lockOut(mMutexOut);

    //if(!mbBad){
       /* cv::Mat Tcw_ = cv::Mat(4, 4, 5);
        Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KFred::_mTcw_type, float>(Tcw_, msg->mTcw);
        SetPose(Tcw_);*/
        if(mSysState == eSystemState::SERVER)
        {
          //  unique_lock<mutex> lock2(mMutexBadFlag);
            if(msg->mTcpred[0]==0 && msg->mTcpred[3]==0){
                std::cout<<"reduced msg bad kf:"<<mnId<<std::endl;
                return;
            }
            if(!mbPoseLock)
            {
                if(!mbIsInit)
                {
                    bool bSetPose = SetPoseFromMessage(msg,mg2oS_wcurmap_wclientmap);
                    if(!bSetPose)
                    {
                      /*  mbOmitSending = false;*///TODOs
                        return;
                    }
                }
                else
                {
                    //init kf shouldnot be updated, DO NOTHING

                }
            }
            /*int numMP = 0;
            int numNotFound = 0;
            int numBesitzt = 0;
            int numAdded = 0;
            for(int idx=0;idx<msg->mvpMapPoints_Ids.size();++idx)
            {
                size_t FeatId = msg->mvpMapPoints_VectId[idx];
                MapPoint* pMP = mpMap->GetMapPointWithId(
                    msg->mvpMapPoints_ClientIds[idx], msg->mvpMapPoints_Ids[idx]
                );
                if(pMP)
                {
                    numMP++;
                    if(mvpMapPoints[FeatId]){
                        if(mvpMapPoints[FeatId]==pMP){
                            //do nothing
                            continue;
                        }
                        else{
                            ++numBesitzt;
                        }
                    }
                    else{
                        mvpMapPoints[FeatId] = pMP;
                        ++numAdded;
                    }
                }
                else
                {
                    numNotFound++;
                }

            }*/
            /*std::cout<<"UPDATE:number of not found mp:"<<numNotFound
            <<", found:"<<numMP<<",numBesitzt:"<<numBesitzt<<", Added:"<<numAdded<<std::endl;
            */
            mbBad = false; //once it has new pose , its again a good kf
        }
        else if(mSysState == eSystemState::CLIENT)
        {
            if(msg->mTcpred[0]==0 && msg->mTcpred[3]==0){
                std::cout<<"msg bad kf:"<<mnId<<std::endl;
                return;
            }

            //!!!!!!!!!!!!!!!!!!!!!!!
            if(!msg->mbPoseLock)
                return;

            if(bUpdateMPs){
                //CHECK IF THERE IS mvpMapPoints_Ids...
                if(!msg->mvpMapPoints_Ids[0] || msg->mvpMapPoints_Ids.size()==0){
                    std::cout<<"2WARN: NO mvpMapPoints_Ids"<<std::endl;
                    return;
                }
                //std::cout<<"Update MPs for KF from another"<<std::endl;
                int numMP = 0;
                for(int idx=0;idx<msg->mvpMapPoints_Ids.size();++idx)
                {
                    size_t FeatId = msg->mvpMapPoints_VectId[idx];
                    MapPoint* pMP = mpMap->GetMapPointWithId(
                        msg->mvpMapPoints_ClientIds[idx], msg->mvpMapPoints_Ids[idx]
                    );
                    if(pMP)
                    {
                        if(mvpMapPoints[FeatId]){
                            continue;
                            //if(mvpMapPoints[FeatId]!=pMP)
                                //std::cout<<"mp is already there, no need to update"<<std::endl;
                            //else
                                //std::cout<<"another mp is already there, what should we do"<<std::endl;
                        }
                        else{
                            //std::cout<<"add mp to this kf"<<std::endl;
                            mvpMapPoints[FeatId] = pMP;
                            numMP++;
                        }
                    }
                    else
                    {
                        //std::cout<<"map point not found!"<<std::endl;
                        //if MP not in Map, we ignore it. Might be deleted, or comes in later and is then (hopefully) added to this KF
                        //TONGJIANG:really hopefully....but actually not....
                    }
                }
                if(numMP==0)
                    std::cout<<"UPDATE MP:no new mp!!!"<<std::endl;
            }
            else{
                bool bSetPose = SetPoseFromMessage(msg);
                if(!bSetPose)
                {
                    return;
                }
            }
        }
    //}
  //  mbOmitSending = false;
}
void KeyFrame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(mnGridCols*mnGridRows);
    mGrid.resize(mnGridCols);
    for(unsigned int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for (unsigned int j=0; j<mnGridRows;j++)
            mGrid[i][j].reserve(nReserve);
    }

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

bool KeyFrame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=mnGridCols || posY<0 || posY>=mnGridRows)
        return false;

    return true;
}


bool KeyFrame::SetPoseFromMessage(orb_slam3_ros::KF *msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    orb_slam3_ros::KFred *pMsgRed = new orb_slam3_ros::KFred();

    ReduceMessage(msg,pMsgRed);

    bool bReturn = SetPoseFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;

    return bReturn;
}

bool KeyFrame::SetPoseFromMessage(orb_slam3_ros::KFred *msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState == eSystemState::SERVER)
    {
      //  unique_lock<mutex> lock2(mMutexBadFlag);
        if(msg->mTcpred[3]==0 && msg->mTcpar[3]==0){
            std::cout<<"KF:03"<<std::endl;
            return false;
        }

        // SERVER
        if(mnId != 0)
        {
            cv::Mat tempTcp = cv::Mat(4,4,5);
            cv::Mat tempVcp = cv::Mat(3,1,CV_32F); //FOR IMU
            KeyFrame* pRef = nullptr;
            if(mSensor==eSensor::MONOCULAR){
                pRef = mpMap->GetKeyFrameWithId(msg->mpPred_KfClientId, msg->mpPred_KfId);

                if(pRef)
                {
                    Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcpred_type,float>(tempTcp,msg->mTcpred);
                    if(tempTcp.at<float>(0,3)==0)
                        std::cout<<"pRef tempTcp"<<tempTcp.at<float>(0,3)<<",";
                }
                else{
                    std::cout<<"KF"<<msg->mnId<<"'s pred"<<msg->mpPred_KfId<<" not found"<<std::endl;
                    if(mpMap->IsKfErased(msg->mpPred_KfClientId, msg->mpPred_KfId))
                    {
                        std::cout<<"Comm:: the Pred kf"<< msg->mpPred_KfId <<"was deleted"<<std::endl;
                    }
                }

                if(!pRef || pRef->isBad())
                {
                    pRef = mpMap->GetKeyFrameWithId(msg->mpPredPred_KfClientId, msg->mpPredPred_KfId);
                    if(pRef)
                    {
                        Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcpredpred_type,float>(tempTcp,msg->mTcpredpred);
                    }
                    else{
                        std::cout<<"KF"<<msg->mnId<<"'s predpred"<<msg->mpPredPred_KfId<<" not found"<<std::endl;
                        if(mpMap->IsKfErased(msg->mpPredPred_KfClientId, msg->mpPredPred_KfId))
                        {
                            std::cout<<"Comm:: the PredPred kf"<< msg->mpPredPred_KfId <<"was deleted"<<std::endl;
                        }
                    }

                }

                if(!pRef|| pRef->isBad())
                {
                    pRef = mpMap->GetKeyFrameWithId(msg->mpPar_KfClientId, msg->mpPar_KfId);

                    if(pRef)
                    {
                        Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcpar_type,float>(tempTcp,msg->mTcpar);
                    }
                    else{
                        if(mpMap->IsKfErased(msg->mpPar_KfClientId, msg->mpPar_KfId))
                        {
                            std::cout<<"Comm:: the Par kf"<< msg->mpPar_KfId <<"was deleted"<<std::endl;
                        }
                    }
                }
            }

            if (mSensor == IMU_MONOCULAR){
                if(!mPrevKF){
                    mPrevKF = mpMap->GetKeyFrameWithId(msg->mpPrevKfClientId, msg->mpPrevKfId);
                    mPrevKFOri = mPrevKF;
                }
                else{
                    if(mPrevKF->isBad()){
                        std::cout<<"mPrevKF is BAD"<<std::endl;
                        mPrevKF = mpMap->GetKeyFrameWithId(msg->mpPrevKfClientId, msg->mpPrevKfId);
                    }
                }
                if(mPrevKF)
                {
                    if(!mPrevKF->isBad()){
                        if(!mPrevKF->mNextKF)
                            mPrevKF->mNextKF = this;

                        pRef = mPrevKF;
                        Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcprev_type,float>(tempTcp,msg->mTcprev);
                        if(tempTcp.at<float>(0,3)==0)
                          std::cout<<"mPrevKF tempTcp"<<tempTcp.at<float>(0,3)<<",";
                        Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mVcp_type,float>(tempVcp,msg->mVcp);
                        mImuBias = IMU::Bias(msg->mAb[0], msg->mAb[1], msg->mAb[2], msg->mGb[0], msg->mGb[1], msg->mGb[2]);

                        if(!mpImuPreintegrated){
                            mpImuPreintegrated = new IMU::Preintegrated(msg->mImuPreintegrated);
                        }
                        else{
                            orb_slam3_ros::PreintegratedIMU i = msg->mImuPreintegrated;
                            mpImuPreintegrated->Update(i);
                        }
                    }
                }
                else{
                    std::cout<<"KF's previous not found for imu info"<<std::endl;
                    KeyFrame* mErasedKF = mpMap->GetErasedKF(msg->mpPrevKfClientId, msg->mpPrevKfId);
                    if(mErasedKF){
                        std::cout<<"KF:"<<mnId<<"'s previous:"<<mErasedKF->mnId<<"was erased, find its' prev prev"<<std::endl;
                        KeyFrame* mPrevPrev = mErasedKF->mPrevKF;
                        if(mPrevPrev){
                            std::cout<<"prev prev found"<<std::endl;
                        }
                        else{
                            std::cout<<"prev prev also not found"<<std::endl;
                        }
                    }
                }
            }
            if(!pRef)
            {
                std::cout<<RED<<"there is no reference pointer available -- ignore this message"<<RST<<std::endl;
                return false;
            }

            float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale()); //TONGJIANG: mg2oS_wcurmap_wclientmap here ONLY for scale
            s = 1/s; //warn
            if((1./s) == 0)
              std::cout<<RED<<"SCALEERROR"<<RST<<std::endl;
            tempTcp.at<float>(0,3) *=(1./s);
            tempTcp.at<float>(1,3) *=(1./s);
            tempTcp.at<float>(2,3) *=(1./s);

            mmTcPrevOri = tempTcp;
            tempTcp = tempTcp * mPrevKFOri->mmTcPrev;
            cv::Mat tempTcw = cv::Mat(4,4,5);
            cv::Mat tempVw = cv::Mat(3,1,CV_32F);
            if(!pRef->isBad())
            {
                cv::Mat Tpw = pRef->GetPose();
                tempTcw = tempTcp * Tpw;
                SetPose(tempTcw);
                if(mSensor==IMU_MONOCULAR && mPrevKF){
                    tempVw = (tempVcp.t() * pRef->GetRotation()).t();
                    SetVelocity(tempVw);
                }
            }
            else
            {
                mbBad = true;
                return false;
            }
        }
        else
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " must not be called for KF 0" << endl;
            throw infrastructure_ex();
        }
    }
    else if(mSysState == eSystemState::CLIENT){
        cv::Mat tempTcp = cv::Mat(4,4,5);
        //cv::Mat tempVcp = cv::Mat(3,1,CV_32F); //FOR IMU
        cv::Mat tempTcw = cv::Mat(4,4,5);
        //cv::Mat tempVw = cv::Mat(3,1,CV_32F);
        KeyFrame* pRef = nullptr;
        pRef = mpMap->GetKeyFrameWithId(msg->mpPred_KfClientId, msg->mpPred_KfId);

        if(pRef)
        {
            Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTcpred_type,float>(tempTcp,msg->mTcpred);
            if(tempTcp.at<float>(0,3)==0)
                std::cout<<"pRef tempTcp"<<tempTcp.at<float>(0,3)<<",";

            if(!pRef->isBad())
            {
                cv::Mat Tpw = pRef->GetPose();
                //std::cout<<"Tpw:"<<Tpw<<std::endl;
                //std::cout<<"tempTcp:"<<tempTcp<<std::endl;
                tempTcw = tempTcp * Tpw;
                //std::cout<<"tempTcw:"<<tempTcw<<std::endl;
                SetPose(tempTcw, false, true);
            }
            else
            {
                mbBad = true;
                return false;
            }
        }
        else{
            std::cout<<"KF"<<msg->mnId<<"'s ref kf"<<msg->mpPred_KfId<<" not found"<<std::endl;
            return true;//we just do not update it
        }
    }
    else{
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " system can only be SERVER OR CLIENT" << endl;
        throw infrastructure_ex();
    }

    return true;
}

void KeyFrame::ReduceMessage(orb_slam3_ros::KF *pMsgFull, orb_slam3_ros::KFred *pMsgRed)
{
    pMsgRed->mnId = pMsgFull->mnId;
    pMsgRed->mnClientId = pMsgFull->mnClientId;
    pMsgRed->mTcpred = pMsgFull->mTcpred;
    pMsgRed->mTcpar = pMsgFull->mTcpar;
    pMsgRed->mpPred_KfId = pMsgFull->mpPred_KfId;
    pMsgRed->mpPred_KfClientId = pMsgFull->mpPred_KfClientId;
    pMsgRed->mpPredPred_KfId = pMsgFull->mpPredPred_KfId;
    pMsgRed->mpPredPred_KfClientId = pMsgFull->mpPredPred_KfClientId;
    pMsgRed->mpPar_KfId = pMsgFull->mpPar_KfId;
    pMsgRed->mpPar_KfClientId = pMsgFull->mpPar_KfClientId;
    pMsgRed->mbBad = pMsgFull->mbBad;

    if(mSensor==IMU_MONOCULAR){
        pMsgRed->mImuPreintegrated = pMsgFull->mImuPreintegrated;
        pMsgRed->mTcprev = pMsgFull->mTcprev;
        pMsgRed->mVcp = pMsgFull->mVcp;
        pMsgRed->mGb = pMsgFull->mGb;
        pMsgRed->mAb = pMsgFull->mAb;
        pMsgRed->mpPrevKfId = pMsgFull->mpPrevKfId;
        pMsgRed->mpPrevKfClientId = pMsgFull->mpPrevKfClientId;
    }

}

void KeyFrame::SetInit(){
    unique_lock<mutex> lockOut(mMutexOut);
    mbIsInit = true;
}
bool KeyFrame::IsInit(){
    unique_lock<mutex> lockOut(mMutexOut);
    return mbIsInit;
}

void KeyFrame::SetInertialBA1()
{
    unique_lock<mutex> lock(mMutexInertial);
    mbIsGIBA1 = true;
}

bool KeyFrame::GetInertialBA1()
{
    unique_lock<mutex> lock(mMutexInertial);
    return mbIsGIBA1;
}
void KeyFrame::SetInertialBA2()
{
    unique_lock<mutex> lock(mMutexInertial);
    mbIsGIBA2 = true;
}

bool KeyFrame::GetInertialBA2()
{
    unique_lock<mutex> lock(mMutexInertial);
    return mbIsGIBA2;
}

} //namespace ORB_SLAM
