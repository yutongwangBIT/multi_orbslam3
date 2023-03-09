#include "ClientHandler.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "Communicator.h"
#include "LoopClosing.h"
#include "CentralControl.h"
#include "ServerViewer.h"

namespace ORB_SLAM3 {

ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, ORBVocabulary* pVoc, KeyFrameDatabase* pDB, Atlas* pAtlas,
                    uint8_t ClientId, const eSensor sensor, ServerViewer* pViewer, UniqueIdDispenser* pUID, eSystemState SysState)
    : mpVoc(pVoc),mpKFDB(pDB),mpAtlas(pAtlas), mpUID(pUID),
      mNh(Nh),mNhPrivate(NhPrivate), mSensor(sensor),
      mnClientId(ClientId),  mSysState(SysState), mbReset(false), mpViewer(pViewer)
{
    if(mpVoc == nullptr || mpKFDB == nullptr || mpAtlas == nullptr )
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw infrastructure_ex();
    }
    //mpAtlas->msuAssClients.insert(mnClientId); //we don't need, since the clientId is known when atlas constructed

    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transformation

    string camera_type;
    mNhPrivate.param("Server/Camera_type", camera_type, string("PinHole"));
    if(camera_type=="PinHole"){
        float fx, fy, cx, cy;
        mNhPrivate.param("Server/Camera_fx",fx,0.0f);
        mNhPrivate.param("Server/Camera_fy",fy,0.0f);
        mNhPrivate.param("Server/Camera_cx",cx,0.0f);
        mNhPrivate.param("Server/Camera_cy",cy,0.0f);

        std::vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);
    }
    else if(camera_type=="KannalaBrandt8"){
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        mNhPrivate.param("Server/Camera_fx",fx,0.0f);
        mNhPrivate.param("Server/Camera_fy",fy,0.0f);
        mNhPrivate.param("Server/Camera_cx",cx,0.0f);
        mNhPrivate.param("Server/Camera_cy",cy,0.0f);

        mNhPrivate.param("Server/Camera_k1",k1,0.0f);
        mNhPrivate.param("Server/Camera_k2",k2,0.0f);
        mNhPrivate.param("Server/Camera_k3",k3,0.0f);
        mNhPrivate.param("Server/Camera_k4",k4,0.0f);

        vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
        mpCamera = new KannalaBrandt8(vCamCalib);
        std::cout << "- Camera: Fisheye" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << k1 << std::endl;
        std::cout << "- k2: " << k2 << std::endl;
        std::cout << "- k3: " << k3 << std::endl;
        std::cout << "- k4: " << k4 << std::endl;
    }

    mpAtlas->AddCamera(mpCamera);

}

void ClientHandler::InitializeThreads()
{
    InitializeCC();
    //std::cout<<"Initialized CC"<<std::endl;

    if(mSysState == eSystemState::SERVER)
    {
        InitializeServer();
        //std::cout<<"Initialized Server"<<std::endl;
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): the system state should be SERVER instead of: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}

void ClientHandler::InitializeCC()
{
    mpCC = new CentralControl(mNh,mNhPrivate,mnClientId,mSysState, mpCamera,this, mpUID);

    //mpCC->mNativeOdomFrame = "map"; //TODO TONGJIANG
    /*string mncl = to_string(mnClientId);
    mNhPrivate.param(node_name+"/FrameId"+mncl,mpCC->mNativeOdomFrame,std::string("nospec"));
    mpAtlas->mOdomFrame = mpCC->mNativeOdomFrame;*/
    //mpAtlas->AddCCPtr(mpCC); //should here be mpatlas->currentMap or simply mpatlas?what if the new map in atlas is constructed???
    mpAtlas->SetCC(mpCC);
}


void ClientHandler::InitializeServer()
{
    cout << "Client: " << unsigned(mnClientId) << " --> Initialize Threads" << endl;

    //Initialize Loop Closer and start new thread
    mpLoopCloser = new LoopClosing(mpCC, mpAtlas, mpKFDB, mpVoc, mSensor);

    //Initialize Communicator
    mpComm = new Communicator(mpCC,mpVoc,mpAtlas,mpKFDB, mSensor);
    //Initialize LocalMapping, we keep the name, but it actually just processes and sends the kfs and mps
    mpLocalMapper =  new LocalMapping(mpCC, mpAtlas, mpViewer, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO);
    //SET AND NEW THREADS
    mpAtlas->SetCommunicator(mpComm);
    mpLocalMapper->SetCommunicator(mpComm);
    mpComm->SetLocalMapping(mpLocalMapper);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    mptCommunicator = new thread(&Communicator::RunServer, mpComm);
    usleep(10000);
    mptLocalMapping = new thread(&LocalMapping::RunServer, mpLocalMapper);
    usleep(10000);
    mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

    if(mpCC->mpCH == nullptr)
    {
        ROS_ERROR_STREAM("ClientHandler::InitializeThreads()\": mpCC->mpCH is nullptr");
        throw infrastructure_ex();
    }
}

/*void ClientHandler::ChangeAtlas(Atlas* pAtlas, g2o::Sim3 g2oS_wnewmap_wcurmap)
{
    mpAtlas = pAtlas;

    mg2oS_wcurmap_wclientmap = g2oS_wnewmap_wcurmap*mg2oS_wcurmap_wclientmap;
    mpCC->mg2oS_wcurmap_wclientmap = mg2oS_wcurmap_wclientmap;

    bool bLockedComm = mpCC->LockComm(); //should be locked and therefore return false
    bool bLockedMapping = mpCC->LockMapping(); //should be locked and therefore return false

    if(bLockedComm || bLockedMapping)
    {
        if(bLockedComm) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Comm not locked: " << endl;
        if(bLockedMapping) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Mapping not locked: " << endl;
        throw infrastructure_ex();
    }

    mpComm->ChangeAtlas(mpAtlas);
    mpLocalMapper->ChangeAtlas(mpAtlas);
    mpLoopCloser->ChangeAtlas(mpAtlas);
}*/

/*void ClientHandler::SaveMap(const string &path_name) {
        std::cout << "--> Lock System" << std::endl;
        while(!mpCC->LockMapping()){usleep(params::timings::miLockSleep);}
        while(!mpCC->LockComm()){usleep(params::timings::miLockSleep);}
        while(!mpCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
        std::cout << "----> done" << std::endl;

        mpAtlas->SaveMap(path_name);

        std::cout << "--> Unlock System" << std::endl;
        mpCC->UnLockMapping();
        mpCC->UnLockComm();
        mpCC->UnLockPlaceRec();
        std::cout << "----> done" << std::endl;
}

void ClientHandler::SetMapMatcher(matchptr pMatch)
{
    mpAtlasMatcher = pMatch;
    mpComm->SetMapMatcher(mpAtlasMatcher);
    mpLocalMapping->SetMapMatcher(mpAtlasMatcher);
}



void ClientHandler::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}


int ClientHandler::GetNumKFsinLoopFinder()
{
    if(mpLoopFinder)
        return mpLoopFinder->GetNumKFsinQueue();
    else
        return -1;
}

int ClientHandler::GetNumKFsinMapMatcher()
{
    if(mpAtlasMatcher)
        return mpAtlasMatcher->GetNumKFsinQueue();
    else
        return -1;
}

void ClientHandler::ClearCovGraph(size_t MapId)
{
    mpLocalMapping->ClearCovGraph(MapId);
}
*/

} //end ns
