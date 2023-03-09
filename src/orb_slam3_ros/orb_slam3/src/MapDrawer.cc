#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include"Atlas.h"
#include"CentralControl.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3
{
    /*MapDrawer::MapDrawer(Atlas* pAtlas, CentralControl* pCC):
    mpCC(pCC), mNh(pCC->mNhPrivate), mpAtlas(pAtlas)*/
MapDrawer::MapDrawer(Atlas* pAtlas0, Atlas* pAtlas1, Atlas* pAtlas2, Atlas* pAtlas3, CentralControl* pCC):
mpCC(pCC), mNh(pCC->mNhPrivate)
{
    mNh.param("Server/Viewer_KeyFrameSize",mKeyFrameSize,0.05f);
    mNh.param("Server/Viewer_KeyFrameLineWidth",mKeyFrameLineWidth,1.0f);
    mNh.param("Server/Viewer_GraphLineWidth",mGraphLineWidth,0.9f);
    mNh.param("Server/Viewer_PointSize",mPointSize,2.0f);
    mNh.param("Server/Viewer_CameraSize",mCameraSize,0.08f);
    mNh.param("Server/Viewer_CameraLineWidth",mCameraLineWidth,3.0f);
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
}

void MapDrawer::DrawMapPoints()
{
    if(mvpAtlas.empty()) {
        std::cout<<"Warning ServerViewer：mvpAtlas is empty"<<std::endl;
        return;
    }
    if(mvpAtlas.size()==1){
        std::vector<Map*> vpMaps;
        for(Atlas* mpAtlas : mvpAtlas){
            Map* pMap = mpAtlas->GetCurrentMap();

            if(!pMap)
                continue;
            std::vector<Map*>::iterator vit = find(vpMaps.begin(),vpMaps.end(),pMap);
            if(vit == vpMaps.end())
                vpMaps.push_back(pMap);
        }
        if(vpMaps.empty()){
            return;
        }

        for(Map* pMap : vpMaps){
            const vector<MapPoint*> &vpMPs = pMap->GetAllMapPoints();
            if(vpMPs.empty()){
                return;
            }
            else{
                std::cout<<"vpMPs:"<<vpMPs.size()<<std::endl;
            }
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            glColor3f(mfMPColors[0][0],mfMPColors[0][1],mfMPColors[0][2]);
            for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
            {
                if(vpMPs[i]->isBad())
                    continue;
                cv::Mat pos = vpMPs[i]->GetWorldPos();
                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            }
            glEnd();
        }
        vpMaps.clear();
    } 
    else{
        std::vector<Map*> vpMaps;
        for(Atlas* mpAtlas : mvpAtlas){
            Map* pMap = mpAtlas->GetCurrentMap();

            if(!pMap)
                continue;
            std::vector<Map*>::iterator vit = find(vpMaps.begin(),vpMaps.end(),pMap);
            if(vit == vpMaps.end())
                vpMaps.push_back(pMap);
            //else
                //std::cout<<"MapDrawer: another map is already in vpMaps (MapPoint)"<<std::endl;
        }
        if(vpMaps.empty()){
            return;
        }

        for(Map* pMap : vpMaps){
            const vector<MapPoint*> &vpMPs = pMap->GetAllMapPoints();
            if(vpMPs.empty()){
                return;
            }
            else{
                std::cout<<"vpMPs:"<<vpMPs.size()<<std::endl;
            }
            std::vector<int> vpClients=pMap->CountClients();
            int index_color;
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            if(vpClients.empty()){
                std::cout<<"MapDrawer: map is not used by any client?"<<std::endl;
                continue;
            }
            else{
                if(vpClients.size()==1)
                    index_color = vpClients[0];
                else
                    index_color = 4;
            }
            glColor3f(mfMPColors[index_color][0],mfMPColors[index_color][1],mfMPColors[index_color][2]);
            for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
            {
                if(vpMPs[i]->isBad())
                    continue;
                cv::Mat pos = vpMPs[i]->GetWorldPos();
                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            }
            glEnd();
        }
        vpMaps.clear();
    }
/*    for(Atlas* mpAtlas : mvpAtlas){
        const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();*/
  /*  std::vector<size_t> vpMPUniqueIds;
    for(Atlas* mpAtlas : mvpAtlas){
        if(!mpAtlas)
            return;
        if(mpAtlas->GetCurrentMap()==NULL)
            return;
        const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
        if(vpMPs.empty())
            continue;
        std::vector<int> vpClients=mpAtlas->GetMapCountClients();
        int index_color;
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        if(vpClients.empty()){
            std::cout<<"MapDrawer: map is not used by any client?"<<std::endl;
            continue;
        }
        else{
            if(vpClients.size()==1)
                index_color = vpClients[0];
            else
                index_color = 4;
        }
        glColor3f(mfMPColors[index_color][0],mfMPColors[index_color][1],mfMPColors[index_color][2]);
        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad())
                continue;
            std::vector<size_t>::iterator vit = find(vpMPUniqueIds.begin(),vpMPUniqueIds.end(),vpMPs[i]->mnUniqueId);
            if(vit == vpMPUniqueIds.end())
                vpMPUniqueIds.push_back(vpMPs[i]->mnUniqueId);
            else
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
        glEnd();
      }*/
    

        /*if(vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);//TODO
        int nClientId = mpAtlas->mnClientId;
        if(nClientId==0)
            glColor3f(0.0,0.0,0.0); //TODO
        else if(nClientId==1)
            glColor3f(0.1,0.0,0.0);

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad())
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
        glEnd();*/
    //}
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    if(mvpAtlas.empty()) {
        std::cout<<"Warning ServerViewer：mvpAtlas is empty"<<std::endl;
        return;
    }
  /*  std::vector<size_t> vpKFUniqueIds;
    for(Atlas* mpAtlas : mvpAtlas){
        if(!mpAtlas)
            return;
        if(mpAtlas->GetCurrentMap()==NULL)
            return;
        const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
        if(vpKFs.empty())
            continue;
        else
            std::cout<<"atlas for client"<<unsigned(mpAtlas->mnClientId)<<" has kf size:"<<vpKFs.size()<<std::endl;

        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                std::vector<size_t>::iterator vit = find(vpKFUniqueIds.begin(),vpKFUniqueIds.end(),vpKFs[i]->mnUniqueId);
                if(vit == vpKFUniqueIds.end())
                    vpKFUniqueIds.push_back(vpKFs[i]->mnUniqueId);
                else
                    continue;
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();
                if(Twc.at<float>(3,0)==0)
                  std::cout<<"Twc"<<Twc.at<float>(3,0)<<",";
                unsigned int index_color = static_cast<int>(mpAtlas->mnClientId);

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

              //  if(!pKF->GetParent()) // It is the first KF in the map
                if(pKF->mbIsInit || pKF->mbIsVirtualInit)
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);

                    //cout << "Initial KF: " << mpAtlas->GetCurrentMap()->GetOriginKF()->mnId << endl;
                    //cout << "Parent KF: " << vpKFs[i]->mnId << endl;
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    //glColor3f(0.0f,0.0f,1.0f);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();


                glEnd();
            }
        }
    }*/
    std::vector<Map*> vpMaps;
    for(Atlas* mpAtlas : mvpAtlas){
        std::vector<Map*> vpMaps_tmp = mpAtlas->GetAllMaps();
        if(vpMaps_tmp.empty())
            continue;
        for(Map* pMap : vpMaps_tmp){
            std::vector<Map*>::iterator vit = find(vpMaps.begin(),vpMaps.end(),pMap);
            if(vit == vpMaps.end())
            {
                vpMaps.push_back(pMap);
            }
            //else
                //std::cout<<"MapDrawer: another map is already in vpMaps"<<std::endl;
        }
    }
    if(vpMaps.empty()){
        //std::cout<<"vpMaps is empty"<<std::endl;
        return;
    }

    for(size_t k=0; k<vpMaps.size(); ++k){
        Map* pMap = vpMaps[k];
        const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
        if(vpKFs.empty()){
            return;
        }
        else{
            std::cout<<"vpKFs:"<<vpKFs.size()<<std::endl;
        }
      //  std::cout<<"map for client"<<unsigned(pMap->mnClientId)<<" has kf size:"<<vpKFs.size()<<std::endl;
        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();
                /*if(Twc.at<float>(3,0)==0)
                  std::cout<<"Twc"<<Twc.at<float>(3,0)<<",";*/
                unsigned int index_color = k;

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

              //  if(!pKF->GetParent()) // It is the first KF in the map
                if(pKF->IsInit())
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);

                    //cout << "Initial KF: " << mpAtlas->GetCurrentMap()->GetOriginKF()->mnId << endl;
                    //cout << "Parent KF: " << vpKFs[i]->mnId << endl;
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    //glColor3f(0.0f,0.0f,1.0f);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();


                glEnd();
            }
        }
        if(bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            // cout << "-----------------Draw graph-----------------" << endl;
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(50);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if(!vCovKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnUniqueId<vpKFs[i]->mnUniqueId) //to avoid double drawing
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame* pParent = vpKFs[i]->GetParent();
                if(pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
                {
                    if((*sit)->mnUniqueId<vpKFs[i]->mnUniqueId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
                }
            }
            glEnd();
            //ADDED TO VISUALIZE CURRENT CONNECTION
            /*glLineWidth(mGraphLineWidth);
            glColor4f(1.0f,0.0f,0.0f,1.0f);
            glBegin(GL_LINES);

            KeyFrame* cloestKF = mvpAtlas[0]->GetCC()->GetNearestKF();
            if(cloestKF){
                //std::cout<<"close:"<<cloestKF->mnId<<std::endl;
                const vector<KeyFrame*> vCovKFsClose = cloestKF->GetBestCovisibilityKeyFrames(5);
                //std::cout<<"vCovKFsClose size:"<<vCovKFsClose.size()<<std::endl;
                cv::Mat Ow3 = cloestKF->GetCameraCenter();
                int count = 0;
                if(!vCovKFsClose.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFsClose.begin(), vend=vCovKFsClose.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnId==cloestKF->mnId){
                            //std::cout<<"continue"<<std::endl;
                            continue;
                        }
                        cv::Mat Ow4 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow3.at<float>(0),Ow3.at<float>(1),Ow3.at<float>(2));
                        glVertex3f(Ow4.at<float>(0),Ow4.at<float>(1),Ow4.at<float>(2));
                        count++;
                        const vector<KeyFrame*> vCovCov = (*vit)->GetBestCovisibilityKeyFrames(3);
                        if(!vCovCov.empty())
                        {
                            for(vector<KeyFrame*>::const_iterator vcc=vCovCov.begin(), vccend=vCovCov.end();vcc!=vccend;vcc++)
                            {
                                if((*vit)->mnId==(*vcc)->mnId || cloestKF->mnId==(*vcc)->mnId){
                                    //std::cout<<"continue"<<std::endl;
                                    continue;
                                }
                                cv::Mat Ow5 = (*vcc)->GetCameraCenter();
                                glVertex3f(Ow4.at<float>(0),Ow4.at<float>(1),Ow4.at<float>(2));
                                glVertex3f(Ow5.at<float>(0),Ow5.at<float>(1),Ow5.at<float>(2));
                                count++;
                            }
                        }
                    }
                }
                else{
                    std::cout<<"close empty"<<std::endl;
                }
                //std::cout<<"count:"<<count<<std::endl;
            }
            glEnd();*/
        }
    }
    vpMaps.clear();
        /*const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
        std::cout<<"MAPFRAWER: KF size:"<<vpKFs.size()<<", Client:"<<unsigned(mpAtlas->mnClientId)<<std::endl;
        int nClientId = mpAtlas->mnClientId;

        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                if(!pKF->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);

                    //cout << "Initial KF: " << mpAtlas->GetCurrentMap()->GetOriginKF()->mnId << endl;
                    //cout << "Parent KF: " << vpKFs[i]->mnId << endl;
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    //glColor3f(0.0f,0.0f,1.0f);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();*/

                //Draw lines with Loop and Merge candidates
                /*glLineWidth(mGraphLineWidth);
                glColor4f(1.0f,0.6f,0.0f,1.0f);
                glBegin(GL_LINES);
                cv::Mat Ow = pKF->GetCameraCenter();
                const vector<KeyFrame*> vpLoopCandKFs = pKF->mvpLoopCandKFs;
                if(!vpLoopCandKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vpLoopCandKFs.begin(), vend=vpLoopCandKFs.end(); vit!=vend; vit++)
                    {
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }
                const vector<KeyFrame*> vpMergeCandKFs = pKF->mvpMergeCandKFs;
                if(!vpMergeCandKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vpMergeCandKFs.begin(), vend=vpMergeCandKFs.end(); vit!=vend; vit++)
                    {
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }*/

            /*    glEnd();
            }
        }

        if(bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            // cout << "-----------------Draw graph-----------------" << endl;
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if(!vCovKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnId<vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame* pParent = vpKFs[i]->GetParent();
                if(pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
                {
                    if((*sit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
                }
            }

            glEnd();
        }

        if(bDrawInertialGraph && mpAtlas->isImuInitialized())
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(1.0f,0.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            //Draw inertial links
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKFi = vpKFs[i];
                cv::Mat Ow = pKFi->GetCameraCenter();
                KeyFrame* pNext = pKFi->mNextKF;
                if(pNext)
                {
                    cv::Mat Owp = pNext->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }
            }

            glEnd();
        }
*/
        //vector<Map*> vpMaps = mpAtlas->GetAllMaps();

        /*if(bDrawKF)
        {
            for(Map* pMap : vpMaps)
            {
                if(pMap == mpAtlas->GetCurrentMap())
                    continue;

                vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

                for(size_t i=0; i<vpKFs.size(); i++)
                {
                    KeyFrame* pKF = vpKFs[i];
                    cv::Mat Twc = pKF->GetPoseInverse().t();
                    unsigned int index_color = pKF->mnOriginMapId;

                    glPushMatrix();

                    glMultMatrixf(Twc.ptr<GLfloat>(0));

                    if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                    {
                        glLineWidth(mKeyFrameLineWidth*5);
                        glColor3f(1.0f,0.0f,0.0f);
                        glBegin(GL_LINES);
                    }
                    else
                    {
                        glLineWidth(mKeyFrameLineWidth);
                        //glColor3f(0.0f,0.0f,1.0f);
                        glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                        glBegin(GL_LINES);
                    }

                    glVertex3f(0,0,0);
                    glVertex3f(w,h,z);
                    glVertex3f(0,0,0);
                    glVertex3f(w,-h,z);
                    glVertex3f(0,0,0);
                    glVertex3f(-w,-h,z);
                    glVertex3f(0,0,0);
                    glVertex3f(-w,h,z);

                    glVertex3f(w,h,z);
                    glVertex3f(w,-h,z);

                    glVertex3f(-w,h,z);
                    glVertex3f(-w,-h,z);

                    glVertex3f(-w,h,z);
                    glVertex3f(w,h,z);

                    glVertex3f(-w,-h,z);
                    glVertex3f(w,-h,z);
                    glEnd();

                    glPopMatrix();
                }
            }
        }*/
    //}

}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) //TODO
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

        MOw.SetIdentity();
        MOw.m[12] = twc.at<float>(0);
        MOw.m[13] = twc.at<float>(1);
        MOw.m[14] = twc.at<float>(2);
    }
    else
    {
        M.SetIdentity();
        MOw.SetIdentity();
    }
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, pangolin::OpenGlMatrix &MTwwp)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        cv::Mat Rwwp(3,3,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

        MOw.SetIdentity();
        MOw.m[12] = twc.at<float>(0);
        MOw.m[13] = twc.at<float>(1);
        MOw.m[14] = twc.at<float>(2);

        MTwwp.SetIdentity();
        MTwwp.m[0] = Rwwp.at<float>(0,0);
        MTwwp.m[1] = Rwwp.at<float>(1,0);
        MTwwp.m[2] = Rwwp.at<float>(2,0);

        MTwwp.m[4] = Rwwp.at<float>(0,1);
        MTwwp.m[5] = Rwwp.at<float>(1,1);
        MTwwp.m[6] = Rwwp.at<float>(2,1);

        MTwwp.m[8] = Rwwp.at<float>(0,2);
        MTwwp.m[9] = Rwwp.at<float>(1,2);
        MTwwp.m[10] = Rwwp.at<float>(2,2);

        MTwwp.m[12] = twc.at<float>(0);
        MTwwp.m[13] = twc.at<float>(1);
        MTwwp.m[14] = twc.at<float>(2);
    }
    else
    {
        M.SetIdentity();
        MOw.SetIdentity();
        MTwwp.SetIdentity();
    }

}

} //namespace ORB_SLAM
