#ifndef ORB_SLAM3_CLIENT_VIEWER_H_
#define ORB_SLAM3_CLIENT_VIEWER_H_


//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

//ROS
#include <ros/ros.h>
#include <mutex>
#include "MapDrawer.h"
#include "ClientSystem.h"

using namespace std;

namespace ORB_SLAM3{

//forward decs
class MapDrawer;
class FrameDrawer;
class ClientSystem;
//----------

class ClientViewer
{ 
public:
//    ClientViewer(MapDrawer* pMapDrawer0, MapDrawer* pMapDrawer1, MapDrawer* pMapDrawer2, MapDrawer* pMapDrawer3, int NumOfClients);
    ClientViewer(ClientSystem* pSystem, MapDrawer* pMapDrawer, FrameDrawer *pFrameDrawer);
    //Main function
    void RunClient();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:
    ros::NodeHandle mNh;
    int mNumOfClients;
    MapDrawer* mpMapDrawer;
    ClientSystem* mpSystem;
    FrameDrawer* mpFrameDrawer;
/*    MapDrawer* mpMapDrawer0;
    MapDrawer* mpMapDrawer1;
    MapDrawer* mpMapDrawer2;
    MapDrawer* mpMapDrawer3;*/

    // 1/fps in ms
    double mT;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool Stop();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

} //end namespace

#endif
