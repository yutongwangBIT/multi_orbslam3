#include "ServerSystem.h"
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include "LoopClosing.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "CentralControl.h"
#include "Communicator.h"
#include "ClientHandler.h"
#include "CentralControl.h"
#include "ServerViewer.h"
#include "MapDrawer.h"


namespace ORB_SLAM3
{
Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;
ServerSystem::ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const eSensor sensor):
    mNh(Nh), mNhPrivate(NhPrivate), mSensor(sensor), mpViewer(nullptr), mpUID(new UniqueIdDispenser)
{
    string node_name = ros::this_node::getName();
    mNhPrivate.param(node_name+"/NumOfClients",mNumOfClients,0);
    std::cout<<"Number of clients is:"<<mNumOfClients<<std::endl;
    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;

    //----
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Atlas for Clients
    InitializeAtlas();

    //Create Viewer
    InitializeViewer();
}

void ServerSystem::InitializeAtlas(){
   //Atlas for the first CLient
    mpAtlas0 = new Atlas(0, 0,eSystemState::SERVER);
/*    if (mSensor==IMU_STEREO || mSensor == IMU_MONOCULAR){
        mpAtlas->SetInertialSensor();
        mpAtlas->SetImuInitialized();
        mpAtlas->SetInertialBA1();
        mpAtlas->SetInertialBA2();
    }*/
    //Atlas for the second CLient
    if(mNumOfClients > 1){
        mpAtlas1 = new Atlas(0, 1, eSystemState::SERVER);
    }
    else
        mpAtlas1=nullptr;
    //Atlas for the third CLient
    if(mNumOfClients > 2){
        mpAtlas2 = new Atlas(0, 2, eSystemState::SERVER);
    }
    else
        mpAtlas2=nullptr;
    //Atlas for the fourth CLient
    if(mNumOfClients > 3){
        mpAtlas3 = new Atlas(0, 3, eSystemState::SERVER);
    }
    else
        mpAtlas3=nullptr;
}

void ServerSystem::InitializeClients(){
    mpCH0 = new ClientHandler(mNh, mNhPrivate, mpVocabulary, mpKeyFrameDatabase, mpAtlas0, 0, mSensor, mpViewer, mpUID);
    mpCH0->InitializeThreads();
    //std::cout<<"Initialized Client 0"<<std::endl;

    if(mNumOfClients>1){
        mpCH1 = new ClientHandler(mNh, mNhPrivate, mpVocabulary, mpKeyFrameDatabase, mpAtlas1, 1, mSensor, mpViewer, mpUID);
        mpCH1->InitializeThreads();
    }

    if(mNumOfClients>2){
        mpCH2 = new ClientHandler(mNh, mNhPrivate, mpVocabulary, mpKeyFrameDatabase, mpAtlas2, 2, mSensor, mpViewer, mpUID);
        mpCH2->InitializeThreads();
    }

    if(mNumOfClients>3){
        mpCH3 = new ClientHandler(mNh, mNhPrivate, mpVocabulary, mpKeyFrameDatabase, mpAtlas3, 3, mSensor, mpViewer, mpUID);
        mpCH3->InitializeThreads();
    }
}
void ServerSystem::InitializeViewer()
{
    mpCC = new CentralControl(mNh,mNhPrivate,-1,eSystemState::SERVER);
/*    mpMapDrawer0 = new MapDrawer(mpAtlas0, mpCC);
    if(mNumOfClients>1)
        mpMapDrawer1 = new MapDrawer(mpAtlas1, mpCC);
    if(mNumOfClients>2)
        mpMapDrawer2 = new MapDrawer(mpAtlas2, mpCC);
    if(mNumOfClients>3)
        mpMapDrawer3 = new MapDrawer(mpAtlas3, mpCC);
    mpViewer = new ServerViewer(mpMapDrawer0, mpMapDrawer1, mpMapDrawer2, mpMapDrawer3, mNumOfClients);*///TODO
    mpMapDrawer = new MapDrawer(mpAtlas0, mpAtlas1, mpAtlas2, mpAtlas3, mpCC);
    mpViewer = new ServerViewer(mpMapDrawer);
    mptViewer = new thread(&ServerViewer::RunServer, mpViewer);
}

void ServerSystem::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to: " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas0->GetAllMaps();
    std::cout<<"vpMaps size:"<<vpMaps.size()<<std::endl;
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::cout<<"vpKFs size:"<<vpKFs.size()<<std::endl;
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId); //todo, different sort method

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}

} //namespace ORB_SLAM
