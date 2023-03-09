#include "Map.h"
#include "Atlas.h"
#if (CV_MAJOR_VERSION > 3)
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
using namespace cv;
#define CV_LOAD_IMAGE_UNCHANGED IMREAD_UNCHANGED
#endif
#include<mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
}

Map::Map(int initKFid, uint8_t ClientId, eSystemState mSystem):
        mnClientId(ClientId), mSysState(mSystem), mnMaxKFidUnique(0),
        mnInitKFid(initKFid), mnMaxKFid(initKFid),mnLastLoopKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
        mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
        mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false),
        mScale(1.0f)
{
    mnId=nNextId++;
    mRgw = cv::Mat::eye(3, 3, 5);
}
Map::Map(int initKFid, uint8_t ClientId, eSystemState mSystem, unsigned long int nMapId, CentralControl* pCC):
        mnClientId(ClientId), mSysState(mSystem), mnMaxKFidUnique(0),
        mnInitKFid(initKFid), mnMaxKFid(initKFid),mnLastLoopKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
        mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
        mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false),
        mbIMU_BA1(false), mbIMU_BA2(false), mScale(1.0f)
{
    mnId=nMapId;
    mmpIsInUse[mnClientId]=false;
    //mspAtlas.insert(pAtlas);
    if(!pCC)
        std::cout<<"ERROR: no PCC in Map"<<std::endl;
    mspCC.insert(pCC);
    mRgw = cv::Mat::eye(3, 3, 5);
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    mmpMapPoints.clear();
    mmpKeyFrames.clear();
    mmpErasedKeyFrames.clear();

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();

    mspCC.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        //pKF->SetInit();
        mvpKeyFrameOrigins.push_back(pKF);
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    mmpKeyFrames[make_pair(pKF->mnClientId,pKF->mnId)] = pKF;//FINISHED TODO, MAYBE USEFUL TO CLIENT TOO
    if(mSysState==CLIENT){
        if(this->GetInertialBA1())
            pKF->SetInertialBA1();
        if(this->GetInertialBA2())
            pKF->SetInertialBA2();
    }
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
    if(pKF->mnUniqueId>mnMaxKFidUnique)
        mnMaxKFidUnique=pKF->mnUniqueId;

    /*if(mSysState==SERVER){
        std::cout<<"map addkeyframe size:"<<mspKeyFrames.size()<<std::endl;
    }*/
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    mmpMapPoints[make_pair(pMP->mnClientId, pMP->mnId)] = pMP;
}

KeyFrame* Map::GetKeyFrameWithId(uint8_t nClientId, size_t nId){
    unique_lock<mutex> lock(mMutexMap);
    std::map<idpair, KeyFrame*>::iterator mit = mmpKeyFrames.find(make_pair(nClientId, nId));
    if(mit != mmpKeyFrames.end()){
        //Check Client Id
        /*if(unsigned(mit->second->mnClientId)!=unsigned(mnClientId)){
            std::cout<<"Map::GetKeyFrameWithId different Client id:"<<unsigned(mit->second->mnClientId)<<std::endl;
            return nullptr;
        }*/
        return mit->second;
    }
    else
        return nullptr;
}
MapPoint* Map::GetMapPointWithId(uint8_t nClientId, size_t nId){
    unique_lock<mutex> lock(mMutexMap);
    std::map<idpair, MapPoint*>::iterator mit = mmpMapPoints.find(make_pair(nClientId, nId));
    if(mit != mmpMapPoints.end())
        return mit->second;
    else
        return nullptr;
}

KeyFrame* Map::GetPredecessor(KeyFrame* pKF)
{
    KeyFrame* pPred;

    size_t kfid = pKF->mnId;
    while(!pPred)
    {
        kfid--;

        if(kfid == -1)
        {
            cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << __LINE__ << " cannot find predecessor" << endl;
            cout << "KF ID: " << pKF->mnId << "|" << pKF->mnClientId <<",map id:"<<pKF->mnOriginMapId<< endl;
           // cout << "In map: " << this->mnMaxKFid << endl;
            /*cout << "Workaround: take first KF (id 0)" << endl;
            pPred = GetKeyFrameWithId(mnClientId, 0);
            cout << "get KF: 0|" << mnId <<" -- nullptr? " << (int)!pPred << endl;*/
            return nullptr;
        }
        else
        {
            pPred = GetKeyFrameWithId(mnClientId, kfid);
        }
    }

    return pPred;
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexInertial);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexInertial);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    if(mSysState==SERVER){
        unique_lock<mutex> lock2(mMutexErase);
        std::map<idpair,MapPoint*>::iterator mit = mmpMapPoints.find(make_pair(pMP->mnClientId,pMP->mnId));
        if(mit != mmpMapPoints.end()) mmpMapPoints.erase(mit);
        mmpErasedMapPoints[make_pair(pMP->mnClientId, pMP->mnId)] = pMP;
    }
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}
bool Map::IsMpErased(uint8_t clientId, size_t mId){
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErase);
    std::map<idpair, MapPoint*>::iterator mit = mmpErasedMapPoints.find(make_pair(clientId, mId));
    if(mit != mmpErasedMapPoints.end()) return true;
    else return false;
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    //if(mSysState==SERVER){
        unique_lock<mutex> lock2(mMutexErase);
        std::map<idpair,KeyFrame*>::iterator mit = mmpKeyFrames.find(make_pair(pKF->mnClientId,pKF->mnId));
        if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);
        //std::cout<<pKF->mnId<<"is in erased list."<<std::endl;
        mmpErasedKeyFrames[make_pair(pKF->mnClientId,pKF->mnId)] = pKF;
    //}
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

bool Map::IsKfErased(uint8_t clientId, size_t mId){
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErase);
    std::map<idpair, KeyFrame*>::iterator mit = mmpErasedKeyFrames.find(make_pair(clientId, mId));
    if(mit != mmpErasedKeyFrames.end()) return true;
    else return false;
}

KeyFrame* Map::GetErasedKF(uint8_t clientId, size_t mId){
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErase);
    std::map<idpair, KeyFrame*>::iterator mit = mmpErasedKeyFrames.find(make_pair(clientId, mId));
    if(mit != mmpErasedKeyFrames.end()) return mit->second;
    else return nullptr;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames(bool bSort)
{
    unique_lock<mutex> lock(mMutexMap);
    struct compFunctor
    {
        inline bool operator()(KeyFrame* elem1 ,KeyFrame* elem2)
        {
            return elem1->mnId < elem2->mnId;
        }
    };
    vector<KeyFrame*> vKFs(mspKeyFrames.begin(),mspKeyFrames.end());
    if(bSort && mSysState==CLIENT)
    {
        sort(vKFs.begin(), vKFs.end(), compFunctor());
        return vKFs;
    }
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}
long unsigned int Map::GetMaxKFidUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFidUnique;
}

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap(uint8_t nClientId)
{
    mmpIsInUse[nClientId] = true;
}
std::vector<int> Map::CountClients(){
    std::vector<int> vint;
    for(int i=0;i<4;++i){
        if(mmpIsInUse.count(static_cast<uint8_t>(i))){
            vint.push_back(i);
        }
    }
    return vint;
}
void Map::SetStoredMap(uint8_t nClientId)
{
    mmpIsInUse[nClientId] = false;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mnLastLoopKFid = 0;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse(uint8_t nClientId)
{
    return mmpIsInUse[nClientId];
}
bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}

void Map::RotateMap(const cv::Mat &R) //NOT USED
{
    unique_lock<mutex> lock(mMutexMap);

    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    KeyFrame* pKFini = mvpKeyFrameOrigins[0];
    cv::Mat Twc_0 = pKFini->GetPoseInverse();
    cv::Mat Txc_0 = Txw*Twc_0;
    cv::Mat Txb_0 = Txc_0*pKFini->mImuCalib.Tcb;
    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);
    Tyx.rowRange(0,3).col(3) = -Txb_0.rowRange(0,3).col(3);
    cv::Mat Tyw = Tyx*Txw;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy, true);
        cv::Mat Vw = pKF->GetVelocity();
        pKF->SetVelocity(Ryw*Vw);
    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(Ryw*pMP->GetWorldPos()+tyw, true);
        pMP->UpdateNormalAndDepth();
    }
}

void Map::ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel, const bool bLockSend, const cv::Mat t)
{
    //std::cout<<"before before ApplyScaledRotation"<<",";
    unique_lock<mutex> lock(mMutexMap);
    //std::cout<<"before ApplyScaledRotation"<<",";
    //SAVE SCALE AND ROTATION
    mScale = s;
    mRgw = R;
    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    //std::cout<<"R:"<<R<<std::endl;
    //std::cout<<"t:"<<t<<std::endl;
    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Tyw = Tyx*Txw;
    //std::cout<<"Tyw:"<<Tyw<<std::endl;
    Tyw.rowRange(0,3).col(3) = Tyw.rowRange(0,3).col(3)+t;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    // std::cout<<"Ryw:"<<Ryw<<std::endl;
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        //std::cout<<"MAP: pKF:"<<pKF->mnId<<"rotated"<<std::endl;
        cv::Mat Twc = pKF->GetPoseInverse();
        //std::cout<<"Twc:"<<Twc<<std::endl;
        Twc.rowRange(0,3).col(3)*=s;
        cv::Mat Tyc = Tyw*Twc;
        //std::cout<<"Tyc:"<<Tyc<<std::endl;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        cv::Mat Vw = pKF->GetVelocity();
        //std::cout<<"Vw:"<<Vw<<std::endl;
        //std::cout<<"0"<<",";
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);
        //std::cout<<"1"<<",";
        pKF->SetPose(Tcy, false, bLockSend); //CHANGED. ApplyScaledRotation is used for inertial, also need to mark...//EXCHANGED SETPOSE AND SETVELOCITY SEQUENCE FOR COMM
        //std::cout<<"2"<<",";
    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
    //    std::cout<<"3"<<std::endl;
        pMP->SetWorldPos(s*Ryw*pMP->GetWorldPos()+tyw, true, bLockSend);
    //    std::cout<<"4"<<std::endl;
        pMP->UpdateNormalAndDepth();
    //    std::cout<<"5"<<std::endl;
    }
    mnMapChange++;
    std::cout<<"after ApplyScaledRotation"<<std::endl;
}

void Map::ConvertToMessage(orb_slam3_ros::Map &msgMap){
    unique_lock<mutex> lock(mMutexMap);
    //std::cout<<"s:"<<mScale<<std::endl;
    //std::cout<<"R:"<<mRgw<<std::endl;
    msgMap.mScale = mScale;
    Converter::CvMatToMsgArrayFixedSize<orb_slam3_ros::Map::_mRgw_type,float>(mRgw,msgMap.mRgw);
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexInertial);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexInertial);
    return mbIsInertial;
}

void Map::SetInertialBA1()
{
    unique_lock<mutex> lock(mMutexInertial);
    mbIMU_BA1 = true;
}

void Map::SetInertialBA2()
{
    unique_lock<mutex> lock(mMutexInertial);
    mbIMU_BA2 = true;
}

bool Map::GetInertialBA1()
{
    unique_lock<mutex> lock(mMutexInertial);
    return mbIMU_BA1;
}

bool Map::GetInertialBA2()
{
    unique_lock<mutex> lock(mMutexInertial);
    return mbIMU_BA2;
}

void Map::PrintEssentialGraph()
{
    //Print the essential graph
    vector<KeyFrame*> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFrame* pFirstKF;
    for(KeyFrame* pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    cout << "KF: " << pFirstKF->mnId << "/" << unsigned(pFirstKF->mnClientId) << endl;
    set<KeyFrame*> spChilds = pFirstKF->GetChilds();
    cout << "KF has childs size: " << spChilds.size() << endl;
    vector<KeyFrame*> vpChilds;
    vector<string> vstrHeader;
    for(KeyFrame* pKFi : spChilds){
        vstrHeader.push_back("--");
        vpChilds.push_back(pKFi);
    }
    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        string strHeader = vstrHeader[i];
        KeyFrame* pKFi = vpChilds[i];

        cout << strHeader << "KF: " << pKFi->mnId << "/" << unsigned(pKFi->mnClientId)<< endl;

        set<KeyFrame*> spKFiChilds = pKFi->GetChilds();
        cout << "KF has childs size: " << spKFiChilds.size() << endl;
        for(KeyFrame* pKFj : spKFiChilds)
        {
            vpChilds.push_back(pKFj);
            vstrHeader.push_back(strHeader+"--");
        }
    }
    if (count == (mspKeyFrames.size()+10))
        cout << "CYCLE!!"    << endl;

    cout << "------------------" << endl << "End of the essential graph" << endl;
}

bool Map::CheckEssentialGraph(){
    vector<KeyFrame*> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFrame* pFirstKF;
    for(KeyFrame* pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    cout << "Checking if the first KF has parent" << endl;
    if(pFirstKF)
        cout << pFirstKF->mnId << endl;
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }
    set<KeyFrame*> spChilds = pFirstKF->GetChilds();
    vector<KeyFrame*> vpChilds;
    vpChilds.reserve(mspKeyFrames.size());
    for(KeyFrame* pKFi : spChilds)
        vpChilds.push_back(pKFi);
    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        KeyFrame* pKFi = vpChilds[i];
        set<KeyFrame*> spKFiChilds = pKFi->GetChilds();
        for(KeyFrame* pKFj : spKFiChilds)
            vpChilds.push_back(pKFj);
    }

    cout << "count/tot" << count << "/" << mspKeyFrames.size() << endl;
    if (count != (mspKeyFrames.size()-1))
        return false;
    else
        return true;
}

void Map::ChangeId(long unsigned int nId)
{
    std::cout<<"Map: Map:"<<mnId<<" changed its Id to:"<<nId<<std::endl;
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder)
{
    string path_imgs = "./" + name_folder + "/";
    for(KeyFrame* pKFi : lpLocalWindowKFs)
    {
        //cout << "KF " << pKFi->mnId << endl;
        cv::Mat img_i = cv::imread(pKFi->mNameFile, CV_LOAD_IMAGE_UNCHANGED);
        //cout << "Image -> " << img_i.cols << ", " << img_i.rows << endl;
        cv::cvtColor(img_i, img_i, CV_GRAY2BGR);
        //cout << "Change of color in the image " << endl;

        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        int num_points = 0;
        for(int j=0; j<vpMPs.size(); ++j)
        {
            MapPoint* pMPij = vpMPs[j];
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            cv::KeyPoint point_img = pKFi->mvKeysUn[j];
            cv::Point2f reproj_p;
            float u, v;
            bool bIsInImage = pKFi->ProjectPointUnDistort(pMPij, reproj_p, u, v);
            if(bIsInImage){
                //cout << "Reproj in the image" << endl;
                cv::circle(img_i, point_img.pt, 1/*point_img.octave*/, cv::Scalar(0, 255, 0));
                cv::line(img_i, point_img.pt, reproj_p, cv::Scalar(0, 0, 255));
                num_points++;
            }
            else
            {
                //cout << "Reproj out of the image" << endl;
                cv::circle(img_i, point_img.pt, point_img.octave, cv::Scalar(0, 0, 255));
            }

        }
        //cout << "Image painted" << endl;
        string filename_img = path_imgs +  "KF" + to_string(mpCurrentKF->mnId) + "_" + to_string(pKFi->mnId) +  name + "points" + to_string(num_points) + ".png";
        cv::imwrite(filename_img, img_i);
    }

}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this)
            {
                pMPi->EraseObservation(it->first); //We need to find where the KF is set as Bad but the observation is not removed
            }

        }
    }
    cout << "  Bad MapPoints removed" << endl;

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }

    mvpBackupMapPoints.clear();
    // Backup of set container to vector
    //std::copy(mspMapPoints.begin(), mspMapPoints.end(), std::back_inserter(mvpBackupMapPoints));
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Pre-save of mappoint " << pMPi->mnId << endl;
        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }
    cout << "  MapPoints back up done!!" << endl;

    mvpBackupKeyFrames.clear();
    //std::copy(mspKeyFrames.begin(), mspKeyFrames.end(), std::back_inserter(mvpBackupKeyFrames));
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }
    cout << "  KeyFrames back up done!!" << endl;

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    //map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }
    cout << "Number of KF: " << mspKeyFrames.size() << endl;
    cout << "Number of MP: " << mspMapPoints.size() << endl;

    // References reconstruction between different instances
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Post-Load of mappoint " << pMPi->mnId << endl;
        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }
    cout << "End to rebuild MapPoint references" << endl;

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }

    cout << "End to rebuild KeyFrame references" << endl;

    mvpBackupMapPoints.clear();
}
/*Atlas* Map::GetAtlas(){
    unique_lock<mutex> lock(mMutexAtlas);
    return mspAtlas;
}
void Map::InsertAtlas(Atlas* pAtlas){
    unique_lock<mutex> lock(mMutexAtlas);
    //check if patlas already in mspAtlas
    std::vector<Atlas*>::iterator vit = find(mspAtlas.begin(), mspAtlas.end(), pAtlas);
    if(vit != mspAtlas.end())
        std::cout<<"pAtlas is already in map, cannot insert"<<std::endl;
    else
        mspAtlas.insert(pAtlas);
}
void Map::EraseAtlas(Atlas* pAtlas){ //WE MAY NOT USE THIS FUNCTION
    unique_lock<mutex> lock(mMutexAtlas);
    //check if patlas already in mspAtlas
    std::vector<Atlas*>::iterator vit = find(mspAtlas.begin(), mspAtlas.end(), pAtlas);
    if(vit != mspAtlas.end())
        mspAtlas.insert(pAtlas);
    else
        std::cout<<"pAtlas cannont be found in map, cannot erase"<<std::endl;
}*/

void Map::AddCCPtr(CentralControl* pCC){
    unique_lock<mutex> lock(mMutexCC);
    mspCC.insert(pCC);
}
void Map::EraseCCPtr(CentralControl* pCC){
    unique_lock<mutex> lock(mMutexCC);
    //check if pcc is in mspCC
    if(!mspCC.count(pCC))
        std::cout<<"Erasing pCC not in this map"<<std::endl;
    else
        mspCC.erase(pCC);
}
std::set<CentralControl*> Map::GetCCPtrs(){
    unique_lock<mutex> lock(mMutexCC);
    return mspCC;
}
void Map::PackVicinityToMsg(KeyFrame* kfcur, orb_slam3_ros::Map &mapMsg, CentralControl* pCC, uint8_t nClientIdComm)
{
    const int N = 5;
    const int totalMax = 20;
    int count = 0;
    set<KeyFrame*> sKfVicinity;
    set<MapPoint*> sMpVicinity;
    if(!kfcur || !pCC){
        return;
    }
    sKfVicinity.insert(kfcur);
    const vector<KeyFrame*> vCovKFs = kfcur->GetBestCovisibilityKeyFrames(N);
    if(!vCovKFs.empty()){
        for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
        {
            if((*vit)->mnId==kfcur->mnId){
                //std::cout<<"continue"<<std::endl;
                continue;
            }
            if(count<totalMax){
                sKfVicinity.insert(*vit);
                count++;
                const vector<KeyFrame*> vCovCov = (*vit)->GetBestCovisibilityKeyFrames(N);
                for(vector<KeyFrame*>::const_iterator vvit=vCovCov.begin(), vvend=vCovCov.end();vvit!=vvend;vvit++){
                    set<KeyFrame*>::iterator sit = find(sKfVicinity.begin(),sKfVicinity.end(),*vvit);
                    if(sit != sKfVicinity.end())
                        continue;
                    else{
                        if(count<totalMax){
                            sKfVicinity.insert(*vvit);
                            count++;
                        }
                        else
                            goto Stop;
                    }
                }
            }
            else
                goto Stop;
        }
        Stop:;
    }
    //add MPs included by the KFs //TONGJIANG:is it necessary to send all the MapPoints???? What it they are not updated?
    for(set<KeyFrame*>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        vector<MapPoint*> vMPs = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
        {
            MapPoint* pMPi = *vit;
            if(!pMPi || pMPi->isBad()) continue;
            set<MapPoint*>::iterator sit = find(sMpVicinity.begin(),sMpVicinity.end(),pMPi);
            if(sit != sMpVicinity.end())
                continue;
            else
                sMpVicinity.insert(pMPi);
        }
    }
    kfcur->ConvertToMessageServer(mapMsg, kfcur, nClientIdComm);
    for(set<KeyFrame*>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        pKFi->ConvertToMessageServer(mapMsg, kfcur, nClientIdComm);
        if(pKFi->mnClientId!=kfcur->mnClientId){
            std::cout<<"we did send kf of another map"<<std::endl;
        }
    }
    for(set<MapPoint*>::iterator sit = sMpVicinity.begin();sit!=sMpVicinity.end();++sit)
    {
        MapPoint* pMPi = *sit;
        pMPi->ConvertToMessage(mapMsg, kfcur);
        /*if(pMPi->mnOriginMapId!=mnId){
            std::cout<<"we did send old mp"<<std::endl;
        }*/
    }

}
void Map::PackVicinityToMsg2(KeyFrame* kfcur, orb_slam3_ros::Map &mapMsg, CentralControl* pCC, uint8_t nClientIdComm)
{
    const int N1 = 20;
    const int N2 = 5;
    const int totalMax = 50;
    int count = 0;
    set<KeyFrame*> sKfVicinity;
    set<MapPoint*> sMpVicinity;
    if(!kfcur || !pCC){
        return;
    }
    sKfVicinity.insert(kfcur);
    vector<KeyFrame*> vCovKFs = kfcur->GetBestCovisibilityKeyFrames(N1);
    if(vCovKFs.empty())
        return;
    for(vector<KeyFrame*>::iterator vit = vCovKFs.begin();vit!=vCovKFs.end();++vit){
        sKfVicinity.insert(*vit);
    }
    //Get Neighbours/parent/child
    for(set<KeyFrame*>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit){
        //std::cout<<"sKfVicinity size:"<<sKfVicinity.size()<<std::endl;
        if(sKfVicinity.size()>totalMax)
            break;
        KeyFrame* pKF = *sit;
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(N2);
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                set<KeyFrame*>::iterator ssit = find(sKfVicinity.begin(),sKfVicinity.end(),pNeighKF);
                if(ssit == sKfVicinity.end())
                {
                    sKfVicinity.insert(pNeighKF);
                    break;
                }
            }
        }
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        //std::cout<<"spChilds size:"<<spChilds.size()<<std::endl;
        for(set<KeyFrame*>::const_iterator sitchild=spChilds.begin(), sendchild=spChilds.end(); sitchild!=sendchild; sitchild++)
        {
            KeyFrame* pChildKF = *sitchild;
            if(!pChildKF->isBad())
            {
                set<KeyFrame*>::iterator ssit = find(sKfVicinity.begin(),sKfVicinity.end(),pChildKF);
                if(ssit == sKfVicinity.end())
                {
                    //std::cout<<"has child:"<<std::endl;
                    sKfVicinity.insert(pChildKF);
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(!pParent->isBad())
            {
                set<KeyFrame*>::iterator ssit = find(sKfVicinity.begin(),sKfVicinity.end(),pParent);
                if(ssit == sKfVicinity.end())
                {
                    //std::cout<<"has parent:"<<std::endl;
                    sKfVicinity.insert(pParent);
                    break;
                }
            }
        }
        //std::cout<<"3sKfVicinity size:"<<sKfVicinity.size()<<std::endl;
    }
    //std::cout<<"2sKfVicinity size:"<<sKfVicinity.size()<<std::endl;

    //add MPs included by the KFs //TONGJIANG:is it necessary to send all the MapPoints???? What it they are not updated?
    for(set<KeyFrame*>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        vector<MapPoint*> vMPs = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
        {
            MapPoint* pMPi = *vit;
            if(!pMPi || pMPi->isBad()) continue;
            set<MapPoint*>::iterator sit = find(sMpVicinity.begin(),sMpVicinity.end(),pMPi);
            if(sit != sMpVicinity.end())
                continue;
            else
                sMpVicinity.insert(pMPi);
        }
    }
    kfcur->ConvertToMessageServer(mapMsg, kfcur, nClientIdComm);
    for(set<KeyFrame*>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        pKFi->ConvertToMessageServer(mapMsg, kfcur, nClientIdComm);
        /*if(pKFi->mnClientId!=kfcur->mnClientId){
            std::cout<<"we did send kf:"<<pKFi->mnClientId<<"from:"<<
            unsigned(pKFi->mnClientId)<<"to Client:"<<unsigned(kfcur->mnClientId)<<
            "using ref kf:"<<kfcur->mnId<<std::endl;
        }*/
    }
    for(set<MapPoint*>::iterator sit = sMpVicinity.begin();sit!=sMpVicinity.end();++sit)
    {
        MapPoint* pMPi = *sit;
        pMPi->ConvertToMessage(mapMsg, kfcur);
    }

}
} //namespace ORB_SLAM3
