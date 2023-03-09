
#include "ClientSystem.h"
#include "Converter.h"
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
#include "Tracking.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "CentralControl.h"
#include "Communicator.h"
#include "ClientViewer.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"


namespace ORB_SLAM3
{

//Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;


ClientSystem::ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, ORBParameters& parameters,const eSensor sensor, UniqueIdDispenser* pUID,
                        const int initFr, const string &strSequence):
    mNh(Nh), mNhPrivate(NhPrivate),
    mSensor(sensor), mbReset(false), mbResetActiveMap(false), mpUID(pUID),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message

    cout << "OpenCV version : " << CV_VERSION << endl;
    cout << "Major version : " << CV_MAJOR_VERSION << endl;
    cout << "Minor version : " << CV_MINOR_VERSION << endl;
    cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;


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


    bool loadedAtlas = false;

    string name_of_node_ = ros::this_node::getName();
    int ClientId;
    mNhPrivate.param(name_of_node_+"/ClientId",ClientId,-1);
    mnClientId = static_cast<uint8_t>(ClientId);
    cout << "This Client Id is: " << unsigned(mnClientId) << endl;
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

    //Create the Atlas
    //mpMap = new Map();
    /*mpAtlas = new Atlas(0);*/
    mpAtlas = new Atlas(0, mnClientId, eSystemState::CLIENT); //I have made a mistake on the initialization.

    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();

    //Initialize CC
    mpCC = new CentralControl(mNh,mNhPrivate,mnClientId,eSystemState::CLIENT,this,mpUID);
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, nullptr, nullptr, nullptr, mpCC);
    mpViewer = new ClientViewer(this, mpMapDrawer, mpFrameDrawer);
    mptViewer = new thread(&ClientViewer::RunClient, mpViewer);

    mpCommunicator = new Communicator(mpCC, mpVocabulary, mpAtlas, mpKeyFrameDatabase, mSensor);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(mpCC, mnClientId, mpVocabulary, mpFrameDrawer, mpAtlas, mpKeyFrameDatabase, mSensor, parameters, strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpCC, mpAtlas, nullptr, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::RunClient,mpLocalMapper); //Tongjiang TODO:Runclient

    if(!(mSensor==MONOCULAR || mSensor==IMU_MONOCULAR)){
        mpLocalMapper->mInitFr = initFr;
        mpLocalMapper->mThFarPoints = parameters.thDepth;
        if(mpLocalMapper->mThFarPoints!=0)
        {
            cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
            mpLocalMapper->mbFarPoints = true;
        }
        else
            mpLocalMapper->mbFarPoints = false;
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetTracker(mpTracker);
    mpAtlas->SetCommunicator(mpCommunicator);
    mpCommunicator->SetLocalMapping(mpLocalMapper);
    mpLocalMapper->SetCommunicator(mpCommunicator);
    mpTracker->SetCommunicator(mpCommunicator);

    mptCommunicator = new thread(&ORB_SLAM3::Communicator::RunClient, mpCommunicator);

    ROS_INFO("Started Client Node...");
    // Fix verbosity
  //  Verbose::SetTh(Verbose::VERBOSITY_QUIET);
}

cv::Mat ClientSystem::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            cout << "Reset stereo..." << endl;
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp,filename);

    // std::cout << "out grabber" << std::endl;

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    current_position_ = Tcw;

    return Tcw;
}

cv::Mat ClientSystem::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }


    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    current_position_ = Tcw;

    return Tcw;
}

cv::Mat ClientSystem::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, const Range *Range_GT, string filename)
{
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }
    else{
        //cout<<"trackmono"<<endl;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "ClientSystem-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    //std::cout<<"before grabimage"<<std::endl;
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp,filename);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  //  std::cout<<"grabimage dauert:"<<std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()<<std::endl;
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    current_position_ = Tcw;

    return Tcw;
}



void ClientSystem::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void ClientSystem::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool ClientSystem::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void ClientSystem::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void ClientSystem::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void ClientSystem::Shutdown()
{
    mpLocalMapper->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished())
    {
        if(!mpLocalMapper->isFinished())
            cout << "mpLocalMapper is not finished" << endl;
        usleep(5000);
    }

}




int ClientSystem::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> ClientSystem::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> ClientSystem::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double ClientSystem::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool ClientSystem::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}
cv::Mat ClientSystem::GetCurrentPosition () {
    return current_position_;
}

//CHANGED!!!
std::vector<MapPoint*> ClientSystem::GetAllMapPoints() {
    return mpAtlas->GetAllMapPoints();
}

std::vector<Map*> ClientSystem::GetAllMaps() {
    return  mpAtlas->GetAllMaps();
}

std::vector<KeyFrame*> ClientSystem::GetAllKeyFrames(){
    return mpAtlas->GetAllKeyFrames();
}

bool ClientSystem::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void ClientSystem::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}


KeyFrame* ClientSystem::GetCurrentRefKFfromTracking()
{
    if(mpTracker->mState < 2)
        return nullptr;
    else
        return mpTracker->GetReferenceKF();
}

void ClientSystem::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to: " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    //std::cout<<"vpMaps size:"<<vpMaps.size()<<std::endl;
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
    //std::cout<<"vpKFs size:"<<vpKFs.size()<<std::endl;
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
