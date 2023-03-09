#include "Atlas.h"

namespace ORB_SLAM3
{

Atlas::Atlas(uint8_t ClientId, eSystemState SysState):
mnClientId(ClientId), mSysState(SysState)
{
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid, uint8_t ClientId, eSystemState SysState):
mnLastInitKFidMap(initKFid),mnClientId(ClientId), mSysState(SysState)//, mbLockViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    if(mSysState==CLIENT)
        CreateNewMap();

    //mmpMaxOriginMapIds = {{0,-1}};
/*    int id = static_cast<int>(mnClientId);*///
    mmpMaxOriginMapIds[static_cast<int>(mnClientId)]=-1;
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId <<" for Client:"<<unsigned(mnClientId)<< endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap, mnClientId, mSysState);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::CreateNewMapServer(unsigned long int mnInitKFId, unsigned long int mapId, uint8_t nClientId){
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << mapId <<" for Client:"<<unsigned(mnClientId)<< endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        /*if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum*/
        mpCurrentMap->SetStoredMap(mnClientId);
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;
    }
    mpCurrentMap = new Map(mnInitKFId, mnClientId, mSysState, mapId, mpCC);
    mpCurrentMap->SetCurrentMap(mnClientId);
    mspMaps.insert(mpCurrentMap);
    //CHANGED FOR COMMUNICATION
    /*if(mnClientId==nClientId){
        mnMaxOriginMapId = mpCurrentMap->GetId();
        std::cout<<"TODO:mnMaxOriginMapId:"<<mnMaxOriginMapId<<std::endl; //TODO
    }*/
    mmpMaxOriginMapIds[nClientId]=mapId;
}
long int Atlas::GetMaxOriginMapId(uint8_t nClientId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    //std::cout<<"TODO:mnMaxOriginMapId:"<<mmpMaxOriginMapIds[static_cast<int>(mnClientId)]<<std::endl;
    return mmpMaxOriginMapIds[static_cast<int>(mnClientId)];
}

void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Change from map "<<mpCurrentMap->GetId()<<" of client "<<unsigned(mnClientId)<<
    "to map with id: " << pMap->GetId() <<" of client "<<unsigned(pMap->mnClientId)<< endl;
    std::set<Map*>::iterator sit = find(mspMaps.begin(),mspMaps.end(),pMap);
    if(sit == mspMaps.end()) //pMap belongs not to this atlas
        mspMaps.insert(pMap);
    if(mpCurrentMap){
        for(size_t i=0; i<4; ++i){ //once the current map is set to be removed, its all inuse are set false
            if(mpCurrentMap->IsInUse(static_cast<uint8_t>(i))){
                mpCurrentMap->SetStoredMap(static_cast<uint8_t>(i));
                pMap->SetCurrentMap(static_cast<uint8_t>(i));
            }
        }
    }
    mpCurrentMap = pMap;
//    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}


void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexAtlas);
    Map* pMapKF = pKF->GetMap();
//    std::cout<<"Atlas kf size:"<<pMapKF->KeyFramesInMap()<<std::endl;
    pMapKF->AddKeyFrame(pKF);//Tongjiang changed in map addkeyframe to give kf a mnmapId, so when pk passed to comm, the mnMapId is already there
    //CHANGED FOR COMMUNICATION
    if(mSysState == eSystemState::CLIENT && !mspComm.empty())
    {
        Communicator* pComm = *(mspComm.begin());
        if(pComm==nullptr){
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Atlas::AddkeyFrame(): no Comm" << endl;
            throw infrastructure_ex();
        }
        else{
            pComm->PassKftoComm(pKF, pMapKF->GetInertialBA1());
            //pComm->PassKftoComm(pKF, pMapKF->isImuInitialized());
        }
    }

}


void Atlas::AddMapPoint(MapPoint* pMP)
{
    unique_lock<mutex> lock(mMutexAtlas);
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
    if(mSysState == eSystemState::CLIENT && !mspComm.empty())
    {
        Communicator* pComm = *(mspComm.begin());
        if(pComm==nullptr){
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Atlas::AddkeyFrame(): no Comm" << endl;
            throw infrastructure_ex();
        }
        else{
            pComm->PassMptoComm(pMP, pMapMP->GetInertialBA1());
            //pComm->PassMptoComm(pMP, pMapMP->isImuInitialized());
        }
    }
}

void Atlas::AddCamera(GeometricCamera* pCam)
{
    mvpCameras.push_back(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

KeyFrame* Atlas::GetKeyFrameWithId(uint8_t nClientId,size_t nId){
    unique_lock<mutex> lock(mMutexAtlas);
    KeyFrame* pKF = nullptr;
    for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        if(*it!=mpCurrentMap){
            pKF = (*it)->GetKeyFrameWithId(nClientId, nId);
            if(pKF){
                break;
            }
        }
    }
    return pKF;
}
MapPoint* Atlas::GetMapPointWithId(uint8_t nClientId,size_t nId){
    unique_lock<mutex> lock(mMutexAtlas);
    MapPoint* pMP = nullptr;
    for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        if(*it!=mpCurrentMap){
            std::cout<<"find mp in map:"<<(*it)->mnId<<std::endl;
            pMP = (*it)->GetMapPointWithId(nClientId, nId);
            if(pMP){
                std::cout<<"mp found:"<<pMP->mnId<<std::endl;
                break;
            }
        }
    }
    return pMP;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
    //mspCC.clear();
}

Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap){
        if(mSysState==CLIENT)
            CreateNewMap();
        else
            return NULL;
    }

    while(mpCurrentMap->IsBad()){
        usleep(3000);
    }

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/ //why is this block uncommented?
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

void Atlas::SetInertialBA1()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialBA1();
}
void Atlas::SetInertialBA2()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialBA2();
}

void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    cout << "There are " << spCams.size() << " cameras in the atlas" << endl;
    for(Map* pMi : mvpBackupMaps)
    {
        cout << "Pre-save of map " << pMi->GetId() << endl;
        pMi->PreSave(spCams);
    }
    cout << "Maps stored" << endl;
    for(GeometricCamera* pCam : mvpCameras)
    {
        cout << "Pre-save of camera " << pCam->GetId() << endl;
        if(pCam->GetType() == pCam->CAM_PINHOLE)
        {
            mvpBackupCamPin.push_back((Pinhole*) pCam);
        }
        else if(pCam->GetType() == pCam->CAM_FISHEYE)
        {
            mvpBackupCamKan.push_back((KannalaBrandt8*) pCam);
        }
    }

}

void Atlas::PostLoad()
{
    mvpCameras.clear();
    map<unsigned int,GeometricCamera*> mpCams;
    for(Pinhole* pCam : mvpBackupCamPin)
    {
        //mvpCameras.push_back((GeometricCamera*)pCam);
        mvpCameras.push_back(pCam);
        mpCams[pCam->GetId()] = pCam;
    }
    for(KannalaBrandt8* pCam : mvpBackupCamKan)
    {
        //mvpCameras.push_back((GeometricCamera*)pCam);
        mvpCameras.push_back(pCam);
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    map<long unsigned int, KeyFrame*> mpAllKeyFrameId;
    for(Map* pMi : mvpBackupMaps)
    {
        cout << "Map id:" << pMi->GetId() << endl;
        mspMaps.insert(pMi);
        map<long unsigned int, KeyFrame*> mpKeyFrameId;
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpKeyFrameId, mpCams);
        mpAllKeyFrameId.insert(mpKeyFrameId.begin(), mpKeyFrameId.end());
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }

    cout << "Number KF:" << numKF << "; number MP:" << numMP << endl;
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* mMAPi : mspMaps)
    {
        num += mMAPi->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *mMAPi : mspMaps) {
        num += mMAPi->GetAllMapPoints().size();
    }

    return num;
}

std::vector<int> Atlas::GetMapCountClients(){
  return mpCurrentMap->CountClients();
}

} //namespace ORB_SLAM3
