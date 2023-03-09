
#ifndef CLIENTSYSTEM_H_
#define CLIENTSYSTEM_H_

//C++
#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "ImuTypes.h"
#include "Datatypes.h"
#include "RangeTypes.h"


//CHANGED
#include <ros/ros.h>
#include <ros/time.h>

using namespace std;

namespace ORB_SLAM3{

class Atlas;
class Tracking;
class LocalMapping;
class CentralControl;
class Communicator;

class ClientViewer;
class MapDrawer;
class FrameDrawer;

class ClientSystem
{
public:
    ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, ORBParameters& parameters, const eSensor sensor, UniqueIdDispenser* pUID=nullptr,
    const int initFr = 0, const string &strSequence = std::string());

        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
  //  System(const string &strVocFile, ORBParameters& parameters, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string(), const string &strLoadingFile = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), const Range *Range_GT=nullptr, string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();


    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    cv::Mat GetCurrentPosition (); //for ros
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<Map*> GetAllMaps();
    std::vector<KeyFrame*> GetAllKeyFrames();

    void SaveKeyFrameTrajectoryEuRoC(const string &filename);


    KeyFrame* GetCurrentRefKFfromTracking();

    uint8_t mnClientId;

private:
    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    //Communicator. It subscribes and publishes the ros topics.
    Communicator* mpCommunicator;

    //CentralControl. It manages the mutex.
    CentralControl* mpCC;

    ClientViewer* mpViewer;
    std::thread* mptViewer;
    MapDrawer* mpMapDrawer;
    FrameDrawer* mpFrameDrawer;

    // System threads: Local Mapping, Communicator
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptCommunicator;

    //eSystemState mSysState;
    UniqueIdDispenser* mpUID;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    // Current position
    cv::Mat current_position_;

    //ROS infrastructure
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    g2o::Sim3 mg2oS_wcurmap_wclientmap; //transformation from map into client

};

} //end namespace

#endif
