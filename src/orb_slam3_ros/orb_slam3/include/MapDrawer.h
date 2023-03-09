#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include<pangolin/pangolin.h>
#include <ros/ros.h>
//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include<mutex>
 
namespace ORB_SLAM3
{
class Atlas;
class CentralControl;
class MapPoint;
class KeyFrame;

class MapDrawer
{
public:
    MapDrawer(Atlas* pAtlas0, Atlas* pAtlas1, Atlas* pAtlas2, Atlas* pAtlas3, CentralControl* pCC);
    /*MapDrawer(Atlas* pAtlas, CentralControl* pCC);*/
    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, pangolin::OpenGlMatrix &MTwwp);
    CentralControl* mpCC;
private:
    std::vector<Atlas*> mvpAtlas;
    //Atlas* mpAtlas;
    ros::NodeHandle mNh;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

    float mfFrameColors1[6][3] = {{0.0f, 1.0f, 1.0f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f}};

    float mfTranlationMap[4][3] = {{0.0f, 0.0f, 0.0f},
                                {10.0f, 10.0f, 0.0f},
                                {10.0f, 0.0f, 0.0f},
                                {0.0f, 10.0f, 0.0f}};
    float mfMPColors[5][3] = {{0.0f, 0.0f, 0.0f},
                              {0.8f, 0.8f, 0.0f},
                                {0.5f, 0.5f, 0.0f},
                                {0.4f, 0.0f, 0.0f},
                                {1.0f, 0.5f, 0.0f}};
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
