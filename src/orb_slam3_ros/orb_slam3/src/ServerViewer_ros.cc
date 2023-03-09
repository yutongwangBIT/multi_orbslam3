#include <ServerViewer.h>
#include "CentralControl.h"
#include "Map.h"
#include "Atlas.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM3 {

ServerViewer::ServerViewer(Atlas* pAtlas0, Atlas* pAtlas1, Atlas* pAtlas2, Atlas* pAtlas3, CentralControl* pCC)
    : mpCC(pCC),mNh(pCC->mNhPrivate)
{
    mPubMarker0 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerClient0", 5);
    mPubMarker1 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerClient1", 5);
    mPubMarker2 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerClient2", 5);
    mPubMarker3 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerClient3", 5);
    mPubNonActiveMap0 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerNonActiveMap0", 5);
    mPubNonActiveMap1 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerNonActiveMap1", 5);
    mPubNonActiveMap2 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerNonActiveMap2", 5);
    mPubNonActiveMap3 = mNh.advertise<visualization_msgs::Marker> ("/VisualServerNonActiveMap3", 5);
  //  std::cout<<"ServerViewer Initialized"<<std::endl;
    if(!pAtlas0)
        std::cout<<"Warning ServerViewer：pAtlas0 is nullptr"<<std::endl;
    else
        mvpAtlas.push_back(pAtlas0);
    if(pAtlas1)
        mvpAtlas.push_back(pAtlas1); 
    if(pAtlas2)
        mvpAtlas.push_back(pAtlas2);
    if(pAtlas3)
        mvpAtlas.push_back(pAtlas3);
    count = 0;
}
void ServerViewer::RunServer()
{
    while(1)
    {
        DrawActiveMap();
        DrawNonActiveMap();
        usleep(1000000);
    }
}
void ServerViewer::DrawActiveMap(){
    if(mvpAtlas.empty()) {
        std::cout<<"Warning ServerViewer：mvpAtlas is empty"<<std::endl;
        return;
    }

    for(Atlas* patlas : mvpAtlas){
        std::vector<visualization_msgs::Marker> vMsgs;
        if(!patlas)
            continue;
        const std::vector<KeyFrame*> vpKFs = patlas->GetAllKeyFrames();
        if(vpKFs.empty()){
            continue;
        }
        int nClientId = patlas->mnClientId;
        string odomFrameId = patlas->mOdomFrame;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            visualization_msgs::Marker KeyFrames;
            KeyFrames.header.frame_id = odomFrameId; 
            KeyFrames.header.stamp = ros::Time::now();
            std::stringstream* ss = new stringstream;
            *ss << "Server" << "KFs" << i << "Atlas" << nClientId; //TODO Check
            KeyFrames.ns = ss->str();
            delete ss;
            KeyFrames.id=0;
            KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
            KeyFrames.scale.x=0.01;
            KeyFrames.pose.orientation.w=1.0;
            KeyFrames.action=visualization_msgs::Marker::ADD;
            unsigned int index_color = pKFi->mnOriginMapId;
            KeyFrames.color.r = mfFrameColors[index_color][0];
            KeyFrames.color.g = mfFrameColors[index_color][1];
            KeyFrames.color.b = mfFrameColors[index_color][2];
            KeyFrames.color.a = 1;

            cv::Mat o, p1, p2, p3, p4;
            if(nClientId==0){ //USE DIFFERENT SHAPE TO DEFINE MULTI CLIENT MIGHT BE A WAY
                //Camera is a pyramid. Define in camera coordinate system
                const float d = 0.05;
                o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
                p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
                p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
                p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
                p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);
            }
            else if(nClientId==1){
                const float d = 0.03;
                o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
                p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
                p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
                p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
                p4 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
            }

            cv::Mat Twc = pKFi->GetPoseInverse();
            cv::Mat ow = pKFi->GetCameraCenter();
            cv::Mat p1w = Twc*p1;
            cv::Mat p2w = Twc*p2;
            cv::Mat p3w = Twc*p3;
            cv::Mat p4w = Twc*p4;

            geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
            float fScale = 1;
            msgs_o.x=(fScale)*ow.at<float>(2);
            msgs_o.y=-(fScale)*ow.at<float>(0);
            msgs_o.z=-(fScale)*ow.at<float>(1);
            msgs_p1.x=(fScale)*p1w.at<float>(2);
            msgs_p1.y=-(fScale)*p1w.at<float>(0);
            msgs_p1.z=-(fScale)*p1w.at<float>(1);
            msgs_p2.x=(fScale)*p2w.at<float>(2);
            msgs_p2.y=-(fScale)*p2w.at<float>(0);
            msgs_p2.z=-(fScale)*p2w.at<float>(1);
            msgs_p3.x=(fScale)*p3w.at<float>(2);
            msgs_p3.y=-(fScale)*p3w.at<float>(0);
            msgs_p3.z=-(fScale)*p3w.at<float>(1);
            msgs_p4.x=(fScale)*p4w.at<float>(2);
            msgs_p4.y=-(fScale)*p4w.at<float>(0);
            msgs_p4.z=-(fScale)*p4w.at<float>(1);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p1);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p2);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p3);
            KeyFrames.points.push_back(msgs_o);
            KeyFrames.points.push_back(msgs_p4);
            KeyFrames.points.push_back(msgs_p1);
            KeyFrames.points.push_back(msgs_p2);
            KeyFrames.points.push_back(msgs_p2);
            KeyFrames.points.push_back(msgs_p3);
            KeyFrames.points.push_back(msgs_p3);
            KeyFrames.points.push_back(msgs_p4);
            KeyFrames.points.push_back(msgs_p4);
            KeyFrames.points.push_back(msgs_p1);

            vMsgs.push_back(KeyFrames);
        }
        for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
        {
            visualization_msgs::Marker msg = *vit;
            if(!msg.points.empty())
            {
                if(nClientId == 0)
                    mPubMarker0.publish(msg);
                else if(nClientId == 1)
                    mPubMarker1.publish(msg);
                else if(nClientId == 2)
                    mPubMarker2.publish(msg);
                else if(nClientId == 3)
                    mPubMarker3.publish(msg);
                else
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": ServerViewer mnClientId not in [0,3]" << endl;
            }
        }
    }
}
void ServerViewer::DrawNonActiveMap(){
    if(mvpAtlas.empty()) {
        std::cout<<"Warning ServerViewer：mvpAtlas is empty"<<std::endl;
        return;
    }

    for(Atlas* patlas : mvpAtlas){
        std::vector<visualization_msgs::Marker> vMsgs;
        if(!patlas)
            continue;
        const std::vector<Map*> vpMaps = patlas->GetAllMaps();
        int nClientId = patlas->mnClientId;
        if(vpMaps.empty())
            continue;
        for(Map* pMap:vpMaps){
            if(pMap == patlas->GetCurrentMap())
                continue;
            const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
            if(vpKFs.empty()){
                continue;
            }
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKFi = vpKFs[i];
                //std::cout<<"ServerViewer kf id:"<<pKFi->mnId<<std::endl;
                visualization_msgs::Marker KeyFrames;
                KeyFrames.header.frame_id = "map"; //TODO
                KeyFrames.header.stamp = ros::Time::now();
                std::stringstream* ss = new stringstream;
                *ss << "Server" << "KFs" << i << "Atlas" << unsigned(nClientId)<<"Map"<<pMap->mnId; //TODO Check
                KeyFrames.ns = ss->str();
                delete ss;
                KeyFrames.id=0;
                KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
                KeyFrames.scale.x=0.01;
                KeyFrames.pose.orientation.w=1.0;
                KeyFrames.action=visualization_msgs::Marker::ADD;
                unsigned int index_color = pKFi->mnOriginMapId;
                KeyFrames.color.r = mfFrameColors[index_color][0];
                KeyFrames.color.g = mfFrameColors[index_color][1];
                KeyFrames.color.b = mfFrameColors[index_color][2];
                KeyFrames.color.a = 0.4;
                cv::Mat o, p1, p2, p3, p4;
                if(nClientId==0){ //USE DIFFERENT SHAPE TO DEFINE MULTI CLIENT MIGHT BE A WAY
                    //Camera is a pyramid. Define in camera coordinate system
                    const float d = 0.002;
                    o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
                    p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
                    p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
                    p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
                    p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);
                }

                cv::Mat Twc = pKFi->GetPoseInverse();
                cv::Mat ow = pKFi->GetCameraCenter();
                cv::Mat p1w = Twc*p1;
                cv::Mat p2w = Twc*p2;
                cv::Mat p3w = Twc*p3;
                cv::Mat p4w = Twc*p4;

                geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
                float fScale = 1;
                msgs_o.x=(fScale)*ow.at<float>(2);
                msgs_o.y=-(fScale)*ow.at<float>(0);
                msgs_o.z=-(fScale)*ow.at<float>(1);
                msgs_p1.x=(fScale)*p1w.at<float>(2);
                msgs_p1.y=-(fScale)*p1w.at<float>(0);
                msgs_p1.z=-(fScale)*p1w.at<float>(1);
                msgs_p2.x=(fScale)*p2w.at<float>(2);
                msgs_p2.y=-(fScale)*p2w.at<float>(0);
                msgs_p2.z=-(fScale)*p2w.at<float>(1);
                msgs_p3.x=(fScale)*p3w.at<float>(2);
                msgs_p3.y=-(fScale)*p3w.at<float>(0);
                msgs_p3.z=-(fScale)*p3w.at<float>(1);
                msgs_p4.x=(fScale)*p4w.at<float>(2);
                msgs_p4.y=-(fScale)*p4w.at<float>(0);
                msgs_p4.z=-(fScale)*p4w.at<float>(1);
                KeyFrames.points.push_back(msgs_o);
                KeyFrames.points.push_back(msgs_p1);
                KeyFrames.points.push_back(msgs_o);
                KeyFrames.points.push_back(msgs_p2);
                KeyFrames.points.push_back(msgs_o);
                KeyFrames.points.push_back(msgs_p3);
                KeyFrames.points.push_back(msgs_o);
                KeyFrames.points.push_back(msgs_p4);
                KeyFrames.points.push_back(msgs_p1);
                KeyFrames.points.push_back(msgs_p2);
                KeyFrames.points.push_back(msgs_p2);
                KeyFrames.points.push_back(msgs_p3);
                KeyFrames.points.push_back(msgs_p3);
                KeyFrames.points.push_back(msgs_p4);
                KeyFrames.points.push_back(msgs_p4);
                KeyFrames.points.push_back(msgs_p1);

                vMsgs.push_back(KeyFrames);
            }
        }

        for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
        {
            visualization_msgs::Marker msg = *vit;
            if(!msg.points.empty())
            {
                if(nClientId == 0)
                    mPubNonActiveMap0.publish(msg);
                else if(nClientId == 1)
                    mPubNonActiveMap1.publish(msg);
                else if(nClientId == 2)
                    mPubNonActiveMap2.publish(msg);
                else if(nClientId == 3)
                    mPubNonActiveMap3.publish(msg);
                else
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": ServerViewer mnClientId not in [0,3]" << endl;
            }
        }
    }
}
} //end ns
