#include "Atlas.h"
#include "Communicator.h"

namespace ORB_SLAM3 {

Communicator::Communicator(CentralControl* pCC, ORBVocabulary* pVoc, Atlas* pAtlas, KeyFrameDatabase* pKFDB, eSensor Sensor, bool bLoadedMap)
    : mpCC(pCC), mSensor(Sensor), mpCamera(pCC->mpCamera),
      mNh(pCC->mNh), mNhPrivate(pCC->mNhPrivate),
      mpVoc(pVoc), mpAtlas(pAtlas), mpDatabase(pKFDB),
      mnClientId(pCC->mnClientId), mnMaxKfIdSent(0),mbResetRequested(false), mbIsFirstKfInComm(true),
      mbLoadedMap(bLoadedMap), mpNearestKF(NULL), mpLastNearestKF(NULL)
{
    string node_name = ros::this_node::getName();
    mNhPrivate.param(node_name+"/ClientPubMapBuffer",mPubMapBufferSize,100);
    mNhPrivate.param(node_name+"/ServerSubMapBuffer",mSubMapBufferSize,1000);

    if(mpCC->mSysState == eSystemState::CLIENT){
        mNhPrivate.param(node_name+"/mKfItBoundPub",mKfItBoundPub,40);
        mNhPrivate.param(node_name+"/mKfItBound",mKfItBound,30);
        mNhPrivate.param(node_name+"/mMpItBoundPub",mMpItBoundPub,2500);
        mNhPrivate.param(node_name+"/mMpItBound",mMpItBound,3000);
        mNhPrivate.param(node_name+"/Comm/CommRateClient",CommRateClient,5000);
        //THIS IS CODE REDUDANT>>>>TODO
        if (mSensor== ORB_SLAM3::IMU_MONOCULAR){
          cv::Mat Tbc(4,4, CV_32F);
          XmlRpc::XmlRpcValue TbcConfig;
          if(!mNhPrivate.getParam(node_name+"/Tbc_data", TbcConfig))
              ROS_ERROR("Failed to get parameter from server.");
          int matSize = 4;
          for (int i = 0; i < matSize; i++)
          {
            for (int j = 0; j < matSize; j++)
            {
                XmlRpc::XmlRpcValue tmp_value = TbcConfig[matSize * i + j];
                //std::cout<<double(tmp_value)<<std::endl;
                if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    Tbc.at<float>(i, j) = double(tmp_value);
            }
          }
          float freq, Ng, Na, Ngw, Naw;
          mNhPrivate.param(node_name+"/IMU_Frequency",freq,200.0f);
          mNhPrivate.param(node_name+"/IMU_NoiseGyro",Ng,1.7e-4f);
          mNhPrivate.param(node_name+"/IMU_NoiseAcc",Na,2.0000e-3f);
          mNhPrivate.param(node_name+"/IMU_GyroWalk",Ngw,1.9393e-05f);
          mNhPrivate.param(node_name+"/IMU_AccWalk",Naw,3.0000e-03f);
          const float sf = sqrt(freq);
          mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);
          cout << "mImuCalib Tbc: " << mpImuCalib->Tbc <<  endl;
      }
    }
    else if(mpCC->mSysState == eSystemState::SERVER){
        mNhPrivate.param(node_name+"/mKfItBoundPub",mKfItBoundPub,60);
        mNhPrivate.param(node_name+"/mKfItBound",mKfItBound,400);
        mNhPrivate.param(node_name+"/mMpItBoundPub",mMpItBoundPub,3000);
        mNhPrivate.param(node_name+"/mMpItBound",mMpItBound,12000);
        if (mSensor== ORB_SLAM3::IMU_MONOCULAR){
          cv::Mat Tbc(4,4, CV_32F);
          XmlRpc::XmlRpcValue TbcConfig;
          if(!mNhPrivate.getParam("Server/Tbc_data", TbcConfig))
              ROS_ERROR("Failed to get parameter from server.");
          int matSize = 4;
          for (int i = 0; i < matSize; i++)
          {
            for (int j = 0; j < matSize; j++)
            {
                XmlRpc::XmlRpcValue tmp_value = TbcConfig[matSize * i + j];
                //std::cout<<double(tmp_value)<<std::endl;
                if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    Tbc.at<float>(i, j) = double(tmp_value);
            }
          }
          float freq, Ng, Na, Ngw, Naw;
          mNhPrivate.param("Server/IMU_Frequency",freq,200.0f);
          mNhPrivate.param("Server/IMU_NoiseGyro",Ng,1.7e-4f);
          mNhPrivate.param("Server/IMU_NoiseAcc",Na,2.0000e-3f);
          mNhPrivate.param("Server/IMU_GyroWalk",Ngw,1.9393e-05f);
          mNhPrivate.param("Server/IMU_AccWalk",Naw,3.0000e-03f);
          const float sf = sqrt(freq);
          cout << endl;
          cout << "IMU Tbc: " << Tbc <<  endl;
          cout << "IMU frequency: " << freq << " Hz" << endl;
          cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
          cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
          cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
          cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;
          mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);
         // cout << "mImuCalib Tbc: " << mpImuCalib->Tbc <<  endl;
        //  cout << "mImuCalib Tcb: " << mpImuCalib->Tcb <<  endl;

         mScale =1.0f;
         mScaleLast = 1.0f;
         mRgw = cv::Mat::eye(3,3,5);
      }
    }
    else{
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator: invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
    mOutMapCount = 0;
    globalkfcounter = 0;
    /*mMsgCountLastMapMsg = 0;
    mOutMapCount = 0;
    mServerMapCount = 0;

    mnWeakAckKF = KFRANGE;
    mnWeakAckMP = MPRANGE;

    if(mbLoadedMap) return; //do not register communication infrastructure (publisher/subscriber) when map is loaded */

    //Topics
    std::stringstream* ss;
    string PubMapTopicName, MapInTopicName, SysType;

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";

        //Subscriber
       /* mNhPrivate.param("MapInTopicName",MapInTopicName,std::string("nospec"));
        mSubMap = mNh.subscribe<ccmslam_msgs::Map>(MapInTopicName,params::comm::client::miSubMapBufferSize,boost::bind(&Communicator::MapCbClient,this,_1));*/
        ss = new stringstream;
        *ss << "MapFromServer" << unsigned(mnClientId);
        std::cout<<"topic name:"<<ss->str()<<std::endl;
        mSubMap = mNh.subscribe<orb_slam3_ros::Map>(ss->str(), mSubMapBufferSize,
                                boost::bind(&Communicator::MapSubsciberClient,this,_1));
        //Publisher
        ss = new stringstream;
        *ss << "MapOut" << SysType << unsigned(mnClientId);
        PubMapTopicName = ss->str();
        mPubMap = mNh.advertise<orb_slam3_ros::Map>(PubMapTopicName,mPubMapBufferSize);
    }
    else if(mpCC->mSysState == eSystemState::SERVER)
    {
        SysType = "Server";

        //Subscriber
        ss = new stringstream;
        *ss << "MapFromClient" << unsigned(mnClientId);
        //mNhPrivate.param(ss->str(),MapInTopicName,std::string("nospec"));
        std::cout<<"topic name:"<<ss->str()<<std::endl;
        mSubMap = mNh.subscribe<orb_slam3_ros::Map>(ss->str(),mSubMapBufferSize,boost::bind(&Communicator::MapSubsciberServer,this,_1));
        //Publisher
        ss = new stringstream;
        *ss << "MapOut" << SysType << unsigned(mnClientId);
        PubMapTopicName = ss->str();
        mPubMap = mNh.advertise<orb_slam3_ros::Map>(PubMapTopicName,mPubMapBufferSize);

    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator: invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

  /*  delete ss;*/

  /*  if(MapInTopicName=="nospec" )
    {
        cout << "Client " << mnClientId << " \033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::Communicator(...): bad IN topic name" << endl;
        throw estd::infrastructure_ex();
    }*/

}
void Communicator::RunClient()
{
    while(true)
    {
        if(CheckBufferKfOut()){
            if(1)
            {
                while(!mpCC->LockMapping()){
                    //std::cout<<"Comm:mapping is locked"<<std::endl;
                    usleep(1000);
                }
                while(!mpCC->LockTracking()){
                    usleep(1000);
                    //std::cout<<"Comm:tracking is locked"<<std::endl;
                }
            }
            else
            {
                std::cout<<"else"<<std::endl;
                /*while(!mpMap->LockMapUpdate()){usleep(1000);}*/
            }

            {
                unique_lock<mutex> lock(mMutexBuffersIn);

                if(!mlBufKFin.empty())
                {
                    //std::cout<<"Comm 1"<<",";
                    ProcessKfInClient();
                    //std::cout<<"Comm 2"<<",";
                }

                if(!mlBufMPin.empty())
                {
                    //std::cout<<"Comm3"<<",";
                    ProcessMpInClient();
                    //std::cout<<"Comm 4"<<std::endl;
                }

                /*if(!mlBufErasedKFin.empty())
                {
                    ProcessErasedKfInClient();
                }*/
                //std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
                PublishMapClient();
                //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                //std::cout << "PublishMapClient time = " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "[ms]" << std::endl;
                //std::cout<<"Comm 5"<<std::endl;
            }


            if(1)
            {
                mpCC->UnLockMapping();
                mpCC->UnLockTracking();
                //std::cout<<"Comm: unlocked T & LM"<<std::endl;
            }
            else
            {
                std::cout<<"else2"<<std::endl;
                /*mpMap->UnLockMapUpdate();*/
            }
        }
        /*ResetIfRequested();*/
        usleep(CommRateClient);
    }
}
void Communicator::ApplyScaledRotation(){
    Map* pMap = mpAtlas->GetCurrentMap();
    if(!pMap)
        return;
    if(pMap->GetAllKeyFrames().size()==0)
        return;
    pMap->ApplyScaledRotation(mRgw, mScale);
    std::cout<<"Comm: FINISHED ApplyScaledRotation"<<std::endl;
}
void Communicator::RunServer(){
    while(true){
        while(!mpCC->LockComm()){usleep(1000);}
        //while(!mpAtlas->LockMapUpdate()){usleep(1000);}
        {
            unique_lock<mutex> lock(mMutexScale);
            if(fabs(mScaleLast-mScale)>0.000001)
            {
                std::cout<<"SCALE HAS CHANGED!!!"<<std::endl;
                ApplyScaledRotation();
                mScaleLast = mScale;
            }
        }
        {
            unique_lock<mutex> lock(mMutexBuffersIn);

            if(!mlBufKFin.empty())
            {
                //std::cout<<"Comm 1"<<",";
                ProcessKfInServer();
                //std::cout<<"Comm 2"<<",";
            }

            if(!mlBufMPin.empty())
            {
                //std::cout<<"Comm3"<<",";
                ProcessMpInServer();
                //std::cout<<"Comm 4"<<std::endl;
            }

            if(!mlBufErasedMPin.empty())
            {
                ProcessErasedMpInServer();
            }

            /*if(!mlBufErasedKFin.empty())
            {
                ProcessErasedKfInServer();
            }*/

            PublishMapServer();
        }
        mpCC->UnLockComm();
        //mpAtlas->UnLockMapUpdate();

        /*ResetIfRequested();*/

        usleep(5000);
    }
}
void Communicator::ProcessErasedMpInServer(){
    Map* currMap = mpAtlas->GetCurrentMap();
    int ItCount = 0;
    while(!mlBufErasedMPin.empty())
    {
        size_t mp_id = mlBufErasedMPin.front();
        mlBufErasedMPin.pop_front();
        MapPoint* pMP = currMap->GetMapPointWithId(mnClientId, mp_id);
        if(pMP){
            //std::cout<<"Erase MP!"<<std::endl;
            pMP->SetBadFlag();
            ++ItCount;
        }
        else{
            //std::cout<<"MapPoint to be erased not found!"<<std::endl;
        }
    }
    //std::cout<<"We erased:"<<ItCount<<"MPs in total."<<std::endl;
}
void Communicator::ProcessErasedKfInServer(){
    Map* currMap = mpAtlas->GetCurrentMap();
    int ItCount = 0;
    while(!mlBufErasedKFin.empty())
    {
        size_t kf_id = mlBufErasedKFin.front();
        mlBufErasedKFin.pop_front();
        KeyFrame* pKF = currMap->GetKeyFrameWithId(mnClientId, kf_id);
        if(pKF){
            std::cout<<"Erase KF!"<<std::endl;
            if (mSensor==IMU_MONOCULAR)
            {
                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;
                    //std::cout<<"t:"<<t<<std::endl;
                    if(t<3.)
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                    else{
                        std::cout<<"time between next and prev is:"<<t<<std::endl;
                    }
                }
                else{
                    std::cout<<"there is no prev kf or next kf for kf:"<<pKF->mnId<<std::endl;
                }
            }
            else
            {
                std::cout<<"Pkf:"<<pKF->mnId<<", set bad flag"<<std::endl;
                pKF->SetBadFlag();
            }
            ++ItCount;
        }
        else{
            std::cout<<"KeyFrame to be erased not found!"<<std::endl;
        }
    }
    std::cout<<"We erased:"<<ItCount<<"KFs in total."<<std::endl;
}
void Communicator::ProcessKfInServer(){
    int ItCount = 0;
    //std::cout<<"Comm size:"<<mlBufKFin.size()<<std::endl;
    while (!mlBufKFin.empty() && (ItCount < mKfItBound))
    {
        pair<orb_slam3_ros::KF, orb_slam3_ros::KFred> msgpair = mlBufKFin.front();
        mlBufKFin.pop_front();

        if(msgpair.first.mnClientId != MAPRANGE)
        {
            orb_slam3_ros::KF* pMsg = new orb_slam3_ros::KF();
            *pMsg = msgpair.first;
            {
                if(pMsg->mnOriginMapId>mpAtlas->GetMaxOriginMapId(pMsg->mnClientId))
                {
                    std::cout<<"we should create new map for client:"<<unsigned(mnClientId)<<std::endl;
                    if(pMsg->mbIsInit){
                        std::cout<<"Init KF is in, create new map"<<std::endl;
                        if(mpAtlas->GetCurrentMap()){
                            mpAtlas->GetCurrentMap()->EraseCCPtr(mpCC);
                        }
                        mpAtlas->CreateNewMapServer(pMsg->mnId, pMsg->mnOriginMapId, pMsg->mnClientId);
                        if (mSensor==IMU_STEREO || mSensor == IMU_MONOCULAR){
                            mpAtlas->SetInertialSensor();
                            mpAtlas->SetImuInitialized();
                        //    mpAtlas->SetInertialBA1();
                        //    mpAtlas->SetInertialBA2();
                        }
                    }
                    else{
                        std::cout<<"kf in new map is in, but there is no new map. So we create a new map with this virtual initial kf."<<std::endl;
                        if(mpAtlas->GetCurrentMap()){
                            mpAtlas->GetCurrentMap()->EraseCCPtr(mpCC);
                        }
                        mpAtlas->CreateNewMapServer(pMsg->mnId, pMsg->mnOriginMapId, pMsg->mnClientId);
                        pMsg->mbIsInit = true;
                        pMsg->mbIsVirtualInit = true;
                        if (mSensor==IMU_STEREO || mSensor == IMU_MONOCULAR){
                            mpAtlas->SetInertialSensor();
                            mpAtlas->SetImuInitialized();
                        //    mpAtlas->SetInertialBA1();
                        //    mpAtlas->SetInertialBA2();
                        }
                    }
                }
                else{
                    if(pMsg->mbIsInit){
                        std::cout<<"Init KF is in, but new map is already created"<<std::endl;
                        //pMsg->mbIsInit = false;
                    }
                    if(pMsg->mbIsVirtualInit){
                        std::cout<<"mbIsVirtualInit should not be true"<<std::endl;
                    }
                }
                Map* currMap = mpAtlas->GetCurrentMap();
                if(currMap==NULL)
                    std::cout<<"new map not created of client:"<<unsigned(mpAtlas->mnClientId)<<std::endl;
                KeyFrame* pKF = currMap->GetKeyFrameWithId(pMsg->mnClientId, pMsg->mnId);

                if(pKF)
                    std::cout<<"STRANGE: the comming pKF can be found in current map"<<std::endl;
                else{
                    if(currMap->IsKfErased(pMsg->mnClientId, pMsg->mnId))
                    {
                        //Note: this can happen, if the Ack from server to client gets lost
                        //msAcksMP.insert(pMsg->mnId);
                        std::cout<<"Comm:: the new comming kf was deleted???"<<std::endl;
                        delete pMsg;
                        continue;
                    }
                    //std::cout<<"before before"<<std::endl;
                    if(!mSensor==IMU_MONOCULAR)
                        pKF = new KeyFrame(pMsg, currMap, mpVoc, mpDatabase, this, eSystemState::SERVER, mSensor, mpCamera, mpCC->mpUID->GetId(), mpCC->mg2oS_wcurmap_wclientmap);
                    else
                        pKF = new KeyFrame(pMsg, currMap, mpVoc, mpDatabase, this, eSystemState::SERVER, mSensor, mpCamera, mpCC->mpUID->GetId(), mpCC->mg2oS_wcurmap_wclientmap, *mpImuCalib);
                }

                pKF->EstablishInitialConnections();
                if(pKF->isBad())
                {
                    //this->SetWeakAckKF(pMsg->mnId);
                    //std::cout<<"comm pKF->isBad"<<std::endl;
                    delete pMsg;
                    continue;
                }
                pKF->UpdateConnections();
                mpAtlas->AddKeyFrame(pKF);
                mpLocalMapping->InsertKeyFrame(pKF);
            }
        }
        else if(msgpair.second.mnClientId != MAPRANGE)
        {
            orb_slam3_ros::KFred* pMsg = new orb_slam3_ros::KFred();

            *pMsg = msgpair.second;

            {
                //kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mnClientId);
                Map* currMap = mpAtlas->GetCurrentMap();
                if(currMap==NULL)
                    return;
                KeyFrame* pKF = currMap->GetKeyFrameWithId(pMsg->mnClientId, pMsg->mnId);
                if(pKF){
                   // std::cout<<"The sent KF can be found in the map:"<<pKF->mnId<<std::endl;
                }
                else{
                    if(currMap->IsKfErased(pMsg->mnClientId, pMsg->mnId))
                    {
                        //Note: this can happen, if the Ack from server to client gets lost
                        //msAcksMP.insert(pMsg->mnId);
                        //std::cout<<"Comm:: the kf was deleted"<<std::endl;
                        //when a kf was deleted, just ignore it....
                        delete pMsg;
                        continue;
                    }
                    else{
                    //    std::cout<<"Comm:: the kf was not deleted, but also not found"<<std::endl;
                        delete pMsg;
                        continue;
                    }
                  //  std::cout<<"The sent KF can not be found in the map:"<<currMap->mnId<<", KF originally in map:"<<pMsg->mnOriginMapId<<std::endl;
                  //  std::cout<<"The sent KF clientId :"<<unsigned(pMsg->mnClientId)<<", Comm :"<<unsigned(mnClientId)<<std::endl;
                //    delete pMsg;
                //    continue; //TODO: CAN WE CREATE AND USE IT TOO???BUT NOT SO IMPORTANT
                }
                    //std::cout<<"Strange: KF cannot be found:"<<pMsg->mnId<<std::endl;
                //TODO UPDATE KEYFRAME
                if(pKF->isBad())
                {
                    //this->SetWeakAckKF(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
                pKF->UpdateFromMessage(pMsg, mpCC->mg2oS_wcurmap_wclientmap);
                pKF->UpdateConnections();
            }
        }
        ItCount++;
    }
    //std::cout<<"CLIENT:"<<unsigned(mpAtlas->mnClientId)<<", count:"<<ItCount<<std::endl;
}
void Communicator::ProcessMpInServer()
{
    int ItCount = 0;
    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        pair<orb_slam3_ros::MP, orb_slam3_ros::MPred> msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();
        //std::cout<<"mpred mnClientId:"<<unsigned(msgpair.second.mnClientId)<<std::endl;

        if(msgpair.first.mnClientId != MAPRANGE)
        {
            Map* currMap = mpAtlas->GetCurrentMap();
            if(currMap==NULL)
                return;
            orb_slam3_ros::MP* pMsg = new orb_slam3_ros::MP();
             *pMsg = msgpair.first;

            {
                MapPoint* pMP = currMap->GetMapPointWithId(pMsg->mnClientId, pMsg->mnId);
                if(pMP)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    std::cout<<"Comm::ProcessMpInServer mp in already in map"<<std::endl;
                    delete pMsg;
                    continue;
                }
                if(currMap->IsMpErased(pMsg->mnClientId, pMsg->mnId))
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    //msAcksMP.insert(pMsg->mnId);
                    std::cout<<"Comm:: the new comming mp was deleted???"<<std::endl;
                    delete pMsg;
                    continue;
                }

                pMP = new MapPoint(pMsg, currMap, this, eSystemState::SERVER, mpCC->mpUID->GetId(), mpCC->mg2oS_wcurmap_wclientmap);
                if(pMP->isBad())
                {
                    //this->SetWeakAckKF(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
                pMP->EstablishInitialConnections();
                mpAtlas->AddMapPoint(pMP);
            }

            delete pMsg;
        }
       // else if(msgpair.second.mnClientId != MAPRANGE)
        else
        {
            Map* currMap = mpAtlas->GetCurrentMap();
            if(currMap==NULL)
                return;
            orb_slam3_ros::MPred* pMsg = new orb_slam3_ros::MPred();

            *pMsg = msgpair.second;

            MapPoint* pMP = currMap->GetMapPointWithId(pMsg->mnClientId, pMsg->mnId);

            if(pMP)
            {
               // std::cout<<"The sentOnce MP can be found in the map"<<std::endl;
            }
            else{
                if(currMap->IsMpErased(pMsg->mnClientId, pMsg->mnId))
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    //msAcksMP.insert(pMsg->mnId);
                    //std::cout<<"Comm::mp was deleted"<<std::endl;
                    delete pMsg;
                    continue;
                }
                else{
                    //std::cout<<"Comm::mp was not deleted but also not found"<<std::endl;
                    delete pMsg;
                    continue;
                }
            }
            //TODO UPDATE MAPPOINT
            if(pMP->isBad())
            {
                //this->SetWeakAckKF(pMsg->mnId);
                delete pMsg;
                continue;
            }
            pMP->UpdateFromMessage(pMsg, mpCC->mg2oS_wcurmap_wclientmap);
            pMP->UpdateNormalAndDepth();
            delete pMsg;
        }
        ++ItCount;
    }
}

/*void Communicator::RunServer()
{
    while(true)
    {
        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mnClientId);
        #endif

        while(!mpCC->LockComm()){usleep(params::timings::miLockSleep);}

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mnClientId);
        #endif

        while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mnClientId);
        #endif

        this->PublishMapServer();

        {
            unique_lock<mutex> lock(mMutexBuffersIn);

            if(!mlBufKFin.empty())
            {
                this->ProcessKfInServer();
            }

            if(!mlBufMPin.empty())
            {
                #ifdef TRACELOCK
                mpCC->mpLogger->SetComm(__LINE__,mnClientId);
                #endif

                this->ProcessMpInServer();

                #ifdef TRACELOCK
                mpCC->mpLogger->SetComm(__LINE__,mnClientId);
                #endif
            }
        }

        #ifdef TRACELOCK
        mpCC->mpLogger->SetComm(__LINE__,mnClientId);
        #endif

        mpCC->UnLockComm();
        mpMap->UnLockMapUpdate();

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mnClientId);
        #endif

        ResetIfRequested();

        usleep(params::timings::server::miCommRate);
    }
}*/

/*void Communicator::MapCbClient(ccmslam_msgs::MapConstPtr pMsg)
{
    for(int it = 0; it < pMsg->vAckKFs.size() ; ++it)
    {
        if(mlKfOpenAcks.empty())
        {
            break;
        }

        int IDi = pMsg->vAckKFs[it];
        list<AckPairKF>::iterator lit = mlKfOpenAcks.begin();
        while(lit != mlKfOpenAcks.end())
        {
            AckPairKF APi = *lit;

            if(APi.first == IDi)
            {
                kfptr pKFi = APi.second;
                pKFi->Ack();
                lit = mlKfOpenAcks.erase(lit);
                break;
            }
            else if(APi.first < IDi)
            {
                kfptr pKFi = APi.second;
                pKFi->SetSendFull();
                lit = mlKfOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    //Weak Acks
    size_t nWeakAckKF = pMsg->WeakAckKF;

    if(nWeakAckKF != KFRANGE)
    {
        list<AckPairKF>::iterator lit = mlKfOpenAcks.begin();
        while(lit != mlKfOpenAcks.end())
        {
            AckPairKF APi = *lit;

            if(APi.first <= nWeakAckKF)
            {
                kfptr pKFi = APi.second;
                pKFi->SetSendFull();
                lit = mlKfOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    for(int it = 0; it < pMsg->vAckMPs.size() ; ++it)
    {
        if(mlMpOpenAcks.empty())
        {
            break;
        }

        int IDi = pMsg->vAckMPs[it];
        list<AckPairMP>::iterator lit = mlMpOpenAcks.begin();
        while(lit != mlMpOpenAcks.end())
        {
            AckPairMP APi = *lit;

            if(APi.first == IDi)
            {
                mpptr pMPi = APi.second;
                pMPi->Ack();
                lit = mlMpOpenAcks.erase(lit);
                break;
            }
            if(APi.first < IDi)
            {
                mpptr pMPi = APi.second;
                pMPi->SetSendFull();
                lit = mlMpOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    size_t nWeakAckMP = pMsg->WeakAckMP;

    if(nWeakAckMP != MPRANGE)
    {
        list<AckPairMP>::iterator lit = mlMpOpenAcks.begin();
        while(lit != mlMpOpenAcks.end())
        {
            AckPairMP APi = *lit;

            if(APi.first <= nWeakAckMP)
            {
                mpptr pMPi = APi.second;
                pMPi->SetSendFull();
                lit = mlMpOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    //Pack'em in the input buffers

    if(!pMsg->Keyframes.empty())
    {
        if(pMsg->MapPoints.empty())
        {
            return;
        }

        unique_lock<mutex> lock(mMutexBuffersIn);

        ccmslam_msgs::KF msg = pMsg->Keyframes[0];
        kfptr pRefKf = mpMap->GetKfPtr(msg.mpPred_KfId,msg.mpPred_KfClientId);
        if(!pRefKf)
        {
            return;
        }

        mlBufKFin.clear(); //only use most recent information
        mlBufMPin.clear();

        //Keyframes
        for(int idx=0;idx<pMsg->Keyframes.size();++idx)
        {
            ccmslam_msgs::KF msgFull = pMsg->Keyframes[idx];
            ccmslam_msgs::KFred msgRed;
            msgRed.mnClientId = MAPRANGE;
            mlBufKFin.push_back(make_pair(msgFull,msgRed));
        }

        //MapPoints
        for(int idx=0;idx<pMsg->MapPoints.size();++idx)
        {
            ccmslam_msgs::MP msgFull = pMsg->MapPoints[idx];
            ccmslam_msgs::MPred msgRed;
            msgRed.mnClientId = MAPRANGE;
            mlBufMPin.push_back(make_pair(msgFull,msgRed));
        }
    }
    else
    {
//        cout << "+++++ NO KFs +++++" << endl;
    }
}*/
void Communicator::MapSubsciberClient(orb_slam3_ros::MapConstPtr pMsg)
{
    {
        unique_lock<mutex> lock(mMutexBuffersIn);
        if(pMsg->KFUpdates.size() > 0)
        {
            for(int idx=0;idx<pMsg->KFUpdates.size();++idx)
            {
                orb_slam3_ros::KF msgFull;
                msgFull.mnClientId = MAPRANGE;
                orb_slam3_ros::KFred msgRed = pMsg->KFUpdates[idx];
                mlBufKFin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->Keyframes.size() > 0)
        {
            for(int idx=0;idx<pMsg->Keyframes.size();++idx)
            {
                orb_slam3_ros::KF msgFull = pMsg->Keyframes[idx];
                orb_slam3_ros::KFred msgRed;
                msgRed.mnClientId = MAPRANGE;
                mlBufKFin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->MPUpdates.size() > 0)
        {
            for(int idx=0;idx<pMsg->MPUpdates.size();++idx)
            {
                orb_slam3_ros::MP msgFull;
                msgFull.mnClientId = MAPRANGE;
                orb_slam3_ros::MPred msgRed = pMsg->MPUpdates[idx];
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->MapPoints.size() > 0)
        {
            for(int idx=0;idx<pMsg->MapPoints.size();++idx)
            {
                orb_slam3_ros::MP msgFull = pMsg->MapPoints[idx];
                orb_slam3_ros::MPred msgRed;
                msgRed.mnClientId = MAPRANGE;
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }
        std::cout<<"mlBufKFin size:"<<mlBufKFin.size()<<std::endl;
    }
}
void Communicator::MapSubsciberServer(orb_slam3_ros::MapConstPtr pMsg)
{
    {
        unique_lock<mutex> lock(mMutexBuffersIn);
 /*       std::cout<<"KFUpdates size:"<<pMsg->KFUpdates.size()<<"; KF size: "<<pMsg->Keyframes.size()<<std::endl;
        std::cout<<"MPUpdates size:"<<pMsg->MPUpdates.size()<<"; MapPoints size: "<<pMsg->MapPoints.size()<<std::endl;*/
        if(pMsg->KFUpdates.size() > 0)
        {
            for(int idx=0;idx<pMsg->KFUpdates.size();++idx)
            {
                orb_slam3_ros::KF msgFull;
                msgFull.mnClientId = MAPRANGE;
                orb_slam3_ros::KFred msgRed = pMsg->KFUpdates[idx];
                mlBufKFin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->Keyframes.size() > 0)
        {
            for(int idx=0;idx<pMsg->Keyframes.size();++idx)
            {
                orb_slam3_ros::KF msgFull = pMsg->Keyframes[idx];
                orb_slam3_ros::KFred msgRed;
                msgRed.mnClientId = MAPRANGE;
                mlBufKFin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->MPUpdates.size() > 0)
        {
            for(int idx=0;idx<pMsg->MPUpdates.size();++idx)
            {
                orb_slam3_ros::MP msgFull;
                msgFull.mnClientId = MAPRANGE;
                orb_slam3_ros::MPred msgRed = pMsg->MPUpdates[idx];
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->MapPoints.size() > 0)
        {
            for(int idx=0;idx<pMsg->MapPoints.size();++idx)
            {
                orb_slam3_ros::MP msgFull = pMsg->MapPoints[idx];
                orb_slam3_ros::MPred msgRed;
                msgRed.mnClientId = MAPRANGE;
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }

        if(pMsg->vErasedMPs.size() > 0)
        {
            for(int idx=0;idx<pMsg->vErasedMPs.size();++idx)
            {
                mlBufErasedMPin.push_back(pMsg->vErasedMPs[idx]);
            }
        }

        if(pMsg->vErasedKFs.size() > 0)
        {
            for(int idx=0;idx<pMsg->vErasedKFs.size();++idx)
            {
                mlBufErasedKFin.push_back(pMsg->vErasedKFs[idx]);
            }
        }
    }

    {
        unique_lock<mutex> lock(mMutexNearestKf);
        std::cout<<"close kf id:"<<pMsg->ClosestKf_Id<<","<<unsigned(pMsg->ClosestKf_ClientId)<<std::endl;
        mNearestKfId = make_pair(pMsg->ClosestKf_Id,pMsg->ClosestKf_ClientId);
        if(mpAtlas){
            if(mpAtlas->GetCurrentMap()){
                //std::cout<<"00:"<<std::endl;
                KeyFrame* pKF = mpAtlas->GetCurrentMap()->GetKeyFrameWithId(pMsg->ClosestKf_ClientId, pMsg->ClosestKf_Id);
                if(pKF){
                    mpNearestKF = pKF;
                    //std::cout<<"mpNearestKF id:"<<mpNearestKF->mnId<<std::endl;
                }
                /*else
                    std::cout<<"no"<<std::endl;*/
            }
        }

    }

    {
        unique_lock<mutex> lock(mMutexScale);
        mScale = pMsg->mScale;
        orb_slam3_ros::Map mapMsg;
        mapMsg.mRgw = pMsg->mRgw;
        Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::Map::_mRgw_type, float>(mRgw, mapMsg.mRgw);
        //std::cout<<"Comm mscale:"<<mScale<<",mRgw:"<<mRgw<<std::endl;
    }

}

void Communicator::PublishMapClient()
{
    /*double dTimeDiff = ros::Time::now().toSec() - mdLastTimePub;
    if(dTimeDiff < mdPeriodicTime)
        return;

    mdLastTimePub = ros::Time::now().toSec();

    if(mpMap->KeyFramesInMap()<=params::tracking::miInitKFs) //starting sending not before tracking is initialized stably //in orb, it's 5
        return;*/
    //CHANGED BY TONGJIANG
    Map* pCurMap = mpAtlas->GetCurrentMap();
    //std::cout<<"all kf size:"<<pCurMap->GetAllKeyFrames().size()<<std::endl;
    //std::cout<<"all Mp size:"<<pCurMap->GetAllMapPoints().size()<<std::endl;
    if((mSensor==MONOCULAR && pCurMap->KeyFramesInMap()<=1) ||(mSensor==IMU_MONOCULAR && !pCurMap->isImuInitialized()))
        return;

    unique_lock<mutex> lockOut(mMutexBuffersOut);


    orb_slam3_ros::Map msgMap;

    pCurMap->ConvertToMessage(msgMap); //FOR SCALE AND Rgw
    //std::cout<<"Comm scale:"<<msgMap.mScale<<",Comm Rgw:"<<msgMap.mRgw[0]<<std::endl;

    KeyFrame* pKFFront=static_cast<KeyFrame*>(NULL);

    int ItCount = 0;
    std::cout<<"Comm: buffer kf size:"<<mspBufferKfOut.size()<<std::endl;
    //Keyframes
    {
        KeyFrame* pCurKf;
        set<KeyFrame*>::iterator sit = mspBufferKfOut.begin();

        while(!mspBufferKfOut.empty() && (ItCount < mKfItBoundPub)) //Do not call checkBuffer()-method -- it also need the BufferOutMutex
        {
            if(sit == mspBufferKfOut.end())
                break;

            pCurKf = *sit;

            if(pCurKf->isBad())
            {
                sit = mspBufferKfOut.erase(sit);
                continue;
            }

            int nFullKFs = msgMap.Keyframes.size(); //we need this to determine whether a full KF was added or not //it is zero for the first keyframe
            pCurKf->ConvertToMessage(msgMap);
           /* msgMap.Keyframes.push_back(pCurKf->GetKFMessageClient());*/
            //CHECK CLIENT ID
            if(!msgMap.Keyframes.empty()){
                orb_slam3_ros::KF kf_msg;
                kf_msg = msgMap.Keyframes.front();
                if(unsigned(kf_msg.mnClientId)!=unsigned(mnClientId)){
                    std::cout<<"Client ID in msg is:"<<unsigned(kf_msg.mnClientId)<<"while in kf is:"<<unsigned(pCurKf->mnClientId)<<std::endl;
                }
            }

            pCurKf->UnMarkInOutBuffer();

            if(msgMap.Keyframes.size() > nFullKFs)
            {
                pair<size_t,KeyFrame*> pAck = make_pair(pCurKf->mnId,pCurKf);
                mlKfOpenAcks.push_back(pAck); //Tongjiang: saved for processing the message sending from server

                if(pCurKf->mnId > mnMaxKfIdSent && pCurKf->mnClientId == mnClientId)
                    mnMaxKfIdSent = pCurKf->mnId;
            }

            if(pKFFront==nullptr){
                pKFFront = pCurKf;
            }
            sit = mspBufferKfOut.erase(sit);
            ++ItCount;

            #ifndef HIDEBUFFERLIMITS
            if(ItCount == mKfItBoundPub)
            {
                cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " reached OUTPUT KF iteration limit [buffsize: " << mspBufferKfOut.size() << "]" << endl;
            }
            #endif
        }
    }

    if(pKFFront==nullptr)
    {
        pKFFront = mpKFLastFront;
    }

    if(pKFFront==nullptr)
    {
        return;
    }
    else{
        mpKFLastFront = pKFFront;
    }
    ++globalkfcounter;
/*    std::cout<<"Comm: globalkfcounter:"<<globalkfcounter<<", mnMaxKfIdSent:"<<mnMaxKfIdSent<<std::endl;
    std::cout<<"Comm: msgMap.Keyframes.size:"<<msgMap.Keyframes.size()<<std::endl;
    std::cout<<"Comm: Pub Kf size:"<<ItCount<<std::endl;*/
    ItCount = 0;

    //MapPoints
    {
        MapPoint* pCurMp;
        set<MapPoint*>::iterator sit = mspBufferMpOut.begin();

        while(!mspBufferMpOut.empty() && (ItCount < mMpItBoundPub)) //Do not call checkBuffer()-method -- it also need the BufferOutMutex
        {
            if(sit == mspBufferMpOut.end())
                break;

            pCurMp = *sit;

            if(pCurMp->isBad())
            {
                sit = mspBufferMpOut.erase(sit);
                continue;
            }

            if(!pCurMp->IsSent() && mnMaxKfIdSent < pCurMp->GetMaxObsKFId())
            {
                ++sit;
                continue;
            }

            int nFullMPs = msgMap.MapPoints.size(); //we need this to determine whether a full MP was added or not

            pCurMp->ConvertToMessage(msgMap,pKFFront); //FINISHED TODO:why pkfFront !!!!It is used for calculating the relative pose instead of absolute...
            /*msgMap.MapPoints.push_back(pCurMp->GetMPMessageClient());*/
           /* pCurMp->ConvertToMessage(msgMap);*/

            pCurMp->UnMarkInOutBuffer();

            if(msgMap.MapPoints.size() > nFullMPs)
            {
                pair<size_t,MapPoint*> pAck = make_pair(pCurMp->mnId,pCurMp);
                mlMpOpenAcks.push_back(pAck);
            }

            sit = mspBufferMpOut.erase(sit);
            ++ItCount;

            #ifndef HIDEBUFFERLIMITS
            if(ItCount == mMpItBoundPub)
            {
                cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " reached OUTPUT MP iteration limit -- size: " << mspBufferMpOut.size() << endl;
            }
            #endif
        }
    }
    //std::cout<<"Comm: MP size:"<<ItCount<<std::endl;
    // pub erased mp ids
    {
        for(size_t k=0; k<mspBufferErasedMpIdsOut.size(); ++k){
            msgMap.vErasedMPs.push_back(static_cast<uint16_t>(mspBufferErasedMpIdsOut[k]));
        }
        //std::cout<<"erased mp size:"<<mspBufferErasedMpIdsOut.size()<<std::endl;
        mspBufferErasedMpIdsOut.clear();
    }
    // pub erased kf ids
    {
        if(mspBufferErasedKfIdsOut.size()>0){
            for(size_t k=0; k<mspBufferErasedKfIdsOut.size(); ++k){
                msgMap.vErasedKFs.push_back(static_cast<uint16_t>(mspBufferErasedKfIdsOut[k]));
                std::cout<<"erased kf:"<<mspBufferErasedKfIdsOut[k]<<std::endl;
            }
            mspBufferErasedKfIdsOut.clear();
        }
    }

    KeyFrame* pClosestKF = mpCC->mpCS->GetCurrentRefKFfromTracking();

    if(pClosestKF)
    {
        msgMap.ClosestKf_Id = static_cast<uint16_t>(pClosestKF->mnId);
        msgMap.ClosestKf_ClientId = static_cast<uint8_t>(pClosestKF->mnClientId);
    }
    else
    {
        msgMap.ClosestKf_Id = KFRANGE;
        msgMap.ClosestKf_ClientId = MAPRANGE;
        std::cout<<"there is no reference kf"<<std::endl;
    }
    /*std::cout<<"Comm: mnMaxKfIdSent:"<<mnMaxKfIdSent<<std::endl;
    std::cout<<"Comm: ClosestKf_Id:"<<pClosestKF->mnId<<std::endl;*/
    /*bool bSend = !msgMap.Keyframes.empty() || !msgMap.KFUpdates.empty() || !msgMap.MapPoints.empty() || msgMap.MPUpdates.empty() || msgMap.ClosestKf_Id != KFRANGE;*/
    bool bSend = !msgMap.Keyframes.empty() || !msgMap.MapPoints.empty() ;
    if(bSend)
    {
        ++mOutMapCount;
        msgMap.mMsgId = mOutMapCount;
        msgMap.header.stamp = ros::Time::now();
        //std::cout<<"comm: bsend"<<std::endl;
        mPubMap.publish(msgMap);
    }
    //std::cout<<"comm: i really finished publishing"<<std::endl;
}
void Communicator::PublishMapServer(){
    orb_slam3_ros::Map msgMap;
    Map* pCurMap = mpAtlas->GetCurrentMap();
    if(!pCurMap){
        return;
    }
    if(pCurMap->KeyFramesInMap() > 10) //start after 10 KFs
    {
        unique_lock<mutex> lock(mMutexNearestKf);
        if(mpNearestKF==NULL)
        {
            //start a new attempt to get the pointer
            //std::cout<<"1"<<std::endl;
            KeyFrame* pKF = pCurMap->GetKeyFrameWithId(mNearestKfId.second, mNearestKfId.first);
            if(pKF){
                mpNearestKF = pKF;
            }
            else{
                //std::cout<<"mpNearestKF not found"<<std::endl;
                KeyFrame* erased = pCurMap->GetErasedKF(mNearestKfId.second, mNearestKfId.first);
                if(erased){
                    std::cout<<"mpNearestKF was earsed:"<<erased->mnId<<std::endl;
                }
                return;
            }
        }
        else{
            /*std::cout<<"2"<<std::endl;
            std::cout<<"mnId"<<mpNearestKF->mnId<<std::endl;
            if(!mNearestKfId.first){
                std::cout<<"ha"<<std::endl;
            }
            std::cout<<","<<mNearestKfId.first<<std::endl;*/
            if(mpNearestKF->mnId != mNearestKfId.first)
            {
                //std::cout<<"4"<<std::endl;
                //try again to get ptr
                KeyFrame* pKF = pCurMap->GetKeyFrameWithId(mNearestKfId.second, mNearestKfId.first);
                if(pKF){
                    mpNearestKF = pKF;
                    //std::cout<<"PublishMapServer found:"<<mpNearestKF->mnId<<std::endl;
                }
                else
                {
                    //try the closest ones -- maybe this can be found
                    bool bFound = false;
                    for(size_t itclose = 1; itclose < 10; ++itclose)
                    {
                        pKF = pCurMap->GetKeyFrameWithId(mNearestKfId.second, mNearestKfId.first - itclose);
                        if(pKF)
                        {
                            mpNearestKF = pKF;
                            bFound = true;
                            break;
                        }
                    }
                }
            }
            mpCC->SetNearestKF(mpNearestKF);
            //KeyFrame* k = mpCC->GetNearestKF();

            /*else{
                std::cout<<"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"<<std::endl;
            }*/

            //if we cannot find the current nearest KF, we use the last valid one
            if(mpNearestKF==mpLastNearestKF)
                return;

            if(!mpNearestKF->GetInertialBA2()) //DO NOT SEND BEFORE FULL INTIALIZED
                return;

            std::cout<<"mpNearestKF:"<<mpNearestKF->mnId<<",Cl:"<<unsigned(mpNearestKF->mnClientId)<<std::endl;
            pCurMap->PackVicinityToMsg2(mpNearestKF,msgMap,mpCC,mnClientId);
        }
        mpLastNearestKF = mpNearestKF;
        mPubMap.publish(msgMap);
    }
}
/*
void Communicator::PublishMapServer()
{
    double dTimeDiff = ros::Time::now().toSec() - mdLastTimePub;
    if(dTimeDiff < mdPeriodicTime)
        return;

    ccmslam_msgs::Map msgMap;

    mdLastTimePub = ros::Time::now().toSec();

    //KF Acks
    for(set<size_t>::iterator sit = msAcksKF.begin();sit != msAcksKF.end();)
    {
        size_t id = *sit;
        msgMap.vAckKFs.push_back((uint16_t)id);
        sit = msAcksKF.erase(sit);
    }

    //MP Acks
    for(set<size_t>::iterator sit = msAcksMP.begin();sit != msAcksMP.end();)
    {
        size_t id = *sit;
        msgMap.vAckMPs.push_back((uint32_t)id);
        sit = msAcksMP.erase(sit);
    }

    //Weak Acks
    msgMap.WeakAckKF = (uint16_t)mnWeakAckKF;
    mnWeakAckKF = KFRANGE;

    msgMap.WeakAckMP = (uint32_t)mnWeakAckMP;
    mnWeakAckMP = MPRANGE;

    //fill with vicitity information
    if(mpMap->KeyFramesInMap() > 10) //start after 10 KFs
    {
        unique_lock<mutex> lock(mMutexNearestKf);

        if(!mpNearestKF)
        {
            //start a new attempt to get the pointer
            kfptr pKF = mpMap->GetKfPtr(mNearestKfId);
            if(pKF)
                mpNearestKF = pKF;
        }

        if(mpNearestKF && mNearestKfId.first != KFRANGE)
        {
            if(mpNearestKF->mId != mNearestKfId)
            {
                //try again to get ptr
                kfptr pKF = mpMap->GetKfPtr(mNearestKfId);
                if(pKF)
                    mpNearestKF = pKF;
                else
                {
                    //try the closest ones -- maybe this can be found
                    for(size_t itclose = 1; itclose < 1+SERVERCURKFSEARCHITS; ++itclose)
                    {
                        pKF = mpMap->GetKfPtr(mNearestKfId.first - itclose,mNearestKfId.second);
                        if(pKF)
                        {
                            mpNearestKF = pKF;
                            break;
                        }
                    }
                }
            }

            //if we cannot find the current nearest KF, we use the last valid one
            mpMap->PackVicinityToMsg(mpNearestKF,msgMap,mpCC);
        }
    }

    //publish (if not empty)

    if(!msgMap.vAckKFs.empty() || !msgMap.vAckMPs.empty() || !msgMap.Keyframes.empty() || !msgMap.MapPoints.empty() || !(msgMap.WeakAckKF == KFRANGE) || !(msgMap.WeakAckMP == MPRANGE))
    {
        ++mServerMapCount;
        msgMap.mMsgId = mServerMapCount;
        msgMap.header.stamp = ros::Time::now();

        if(params::comm::server::miKfLimitToClient == 0)
        {
            msgMap.Keyframes.clear();
            msgMap.MapPoints.clear();
            msgMap.KFUpdates.clear();
            msgMap.MPUpdates.clear();
        }

        mPubMap.publish(msgMap);
    }
}
*/
void Communicator::ProcessKfInClient()
{
    int ItCount = 0;
    Map* currMap = mpAtlas->GetCurrentMap();
    if(currMap==NULL)
        return;

    if(!currMap->GetInertialBA2()){
        mlBufKFin.clear();
        return;
    }
    while(!mlBufKFin.empty() && (ItCount < mKfItBound))
    {
        pair<orb_slam3_ros::KF, orb_slam3_ros::KFred> msgpair = mlBufKFin.front();
        mlBufKFin.pop_front();
        if(msgpair.first.mnClientId != MAPRANGE) //ADD NEW KF TO THIS MAP
        {
            orb_slam3_ros::KF* pMsg = new orb_slam3_ros::KF();
            *pMsg = msgpair.first;
            KeyFrame* pKF = currMap->GetKeyFrameWithId(pMsg->mnClientId, pMsg->mnId);

            if(pKF){
                std::cout<<"STRANGE: the comming pKF can be found in current map"<<std::endl;
                delete pMsg;
                continue;
            }
            else{
                std::cout<<"We need create a new kf that is original contructed from client:"<<unsigned(pMsg->mnClientId)<<std::endl;
                if(mSensor==IMU_MONOCULAR)
                    pKF = new KeyFrame(pMsg, currMap, mpVoc, mpDatabase, this, eSystemState::CLIENT, mSensor, nullptr, -100, g2o::Sim3(), *mpImuCalib);
            }

            pKF->EstablishInitialConnections();
            if(pKF->isBad())
            {
                //this->SetWeakAckKF(pMsg->mnId);
                //std::cout<<"comm pKF->isBad"<<std::endl;
                delete pMsg;
                continue;
            }
            pKF->SetFromServer();
            pKF->UpdateConnections();
            currMap->AddKeyFrame(pKF); //use currmap instead of atlas, because atlas addkeyframe send keyframe.
            //mpLocalMapping->InsertKeyFrame(pKF);
            ++ItCount;
        }
        else if(msgpair.second.mnClientId != MAPRANGE)
        {
            orb_slam3_ros::KFred* pMsg = new orb_slam3_ros::KFred();
            *pMsg = msgpair.second;
            //check client id
            //if(pMsg->mnClientId==mnClientId){
                //simply update informaiton???
                KeyFrame* pKF = currMap->GetKeyFrameWithId(pMsg->mnClientId, pMsg->mnId);
                if(!pKF)
                {
                    std::cout<<"Comm:: the kf was not found"<<std::endl;
                    pKF = mpAtlas->GetKeyFrameWithId(pMsg->mnClientId, pMsg->mnId);
                    if(pKF){
                        std::cout<<"Comm:: the kf can be found in an old map!!!!!!!!!!!!"<<std::endl;
                    }
                    else{
                        delete pMsg;
                        continue;
                    }
                }
                //TODO UPDATE KEYFRAME
                if(pKF->isBad())
                {
                    delete pMsg;
                    continue;
                }
                pKF->UpdateFromMessage(pMsg, g2o::Sim3(), mnClientId!=pMsg->mnClientId);
                pKF->UpdateConnections();
            //}
        }

    }
    std::cout<<"ItCount:"<<ItCount<<std::endl;
}

void Communicator::ProcessMpInClient()
{
    int ItCount = 0;
    Map* pCurMap = mpAtlas->GetCurrentMap();
    if(pCurMap==NULL)
        return;

    if(!pCurMap->GetInertialBA2()){
        mlBufMPin.clear();
        return;
    }
    while(!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        pair<orb_slam3_ros::MP, orb_slam3_ros::MPred> msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();
        if(msgpair.first.mnClientId != MAPRANGE)
        {
            orb_slam3_ros::MP* pMsg = new orb_slam3_ros::MP();
            *pMsg = msgpair.first;
            MapPoint* pMP = pCurMap->GetMapPointWithId(pMsg->mnClientId, pMsg->mnId);
            if(pMP){
                //update observation
            //    pMP->UpdateObservation(pMsg, mpAtlas);
            //    pMP->UpdateNormalAndDepth(); // DO NOTHING AT THE MOMENT
            }
            else{
                pMP = mpAtlas->GetMapPointWithId(pMsg->mnClientId, pMsg->mnId);
                if(pMP){
                    std::cout<<"pMP belongs to an old map"<<std::endl;
                    pMP->SetFromServer();
                    pMP->UpdateMap(pCurMap);
                    pCurMap->AddMapPoint(pMP);
                    //ignore EraseMapPoint at this moment
                    pMP->UpdateObservation(pMsg, mpAtlas, true);
                    pMP->UpdateNormalAndDepth();
                }
                else{
                    if(pMsg->mnClientId!=mnClientId){
                        //check if the mp is already there.
                        pMP = pCurMap->GetMapPointWithId(pMsg->mnClientId,pMsg->mnId);
                        if(pMP)
                        {
                            std::cout<<"COMM CLIENT:pMP is already there."<<std::endl;
                            delete pMsg;
                            continue;
                        }
                        //std::cout<<"We need create new mp since a mp from another client is comming"<<std::endl;
                        pMP = new MapPoint(pMsg, pCurMap, this, eSystemState::CLIENT, -100, g2o::Sim3());
                        //pMP->SetFromServer();
                        if(pMP->isBad())
                        {
                            //this->SetWeakAckKF(pMsg->mnId);
                            delete pMsg;
                            continue;
                        }
                        pMP->EstablishInitialConnections();
                        pCurMap->AddMapPoint(pMP);
                        ++ItCount;
                    }
                    else{
                        std::cout<<"The MP might be erased?"<<std::endl;
                    }
                }
            }
        }
        else if(msgpair.second.mnClientId != MAPRANGE)
        {
            std::cout<<"comm: that is not the case."<<std::endl;
        }

    }
    std::cout<<"MP ItCount:"<<ItCount<<std::endl;
}
/*void Communicator::ProcessMpInClient()
{
    int ItCount = 0;

    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        msgMPPair msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();

        if(msgpair.first.mnClientId != MAPRANGE)
        {
            ccmslam_msgs::MP* pMsg = new ccmslam_msgs::MP();

            *pMsg = msgpair.first;

            {
                idpair RefID = make_pair(pMsg->mpPredKFId,pMsg->mpPredKFClientId);
                kfptr pRefKf = mpMap->GetKfPtr(RefID);
                if(!pRefKf)
                {
                    delete pMsg;
                    continue;
                }
            }

            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mnClientId);

            if(pMP)
            {
                if(pMsg->mbPoseChanged)
                {
                    pMP->UpdateFromMessage(pMsg);

                    pMP->UpdateNormalAndDepth();
                }
            }
            else
            {
                pMP.reset(new MapPoint(pMsg,mpMap,shared_from_this(),mpCC->mSysState));

                if(pMP->isBad())
                {
                    delete pMsg;
                    continue;
                }

                pMP->EstablishInitialConnectionsClient();

                if(pMP->isBad())
                {
                    delete pMsg;
                    continue;
                }

                mpMap->AddMapPoint(pMP);
            }

            delete pMsg;
        }
        else if(msgpair.second.mnClientId != MAPRANGE)
        {
            cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Reception of reduced KF not implemented" << endl;
            throw infrastructure_ex();
        }

        ++ItCount;
        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mMpItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mnClientId << ": reached INPUT MP iteration limit" << "[" << mlBufMPin.size() << "]" << endl;
        }
        #endif
    }
}*/

/*void Communicator::ProcessMpInServer()
{
    int ItCount = 0;

    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        msgMPPair msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();

        if(msgpair.first.mnClientId != MAPRANGE)
        {
            ccmslam_msgs::MP* pMsg = new ccmslam_msgs::MP();

            *pMsg = msgpair.first;

            {
                mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mnClientId);
                if(pMP)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    msAcksMP.insert(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
            }
            if(mpMap->IsMpDeleted(pMsg->mnId,pMsg->mnClientId))
            {
                //Note: this can happen, if the Ack from server to client gets lost
                msAcksMP.insert(pMsg->mnId);
                delete pMsg;
                continue;
            }

            mpptr pMP{new MapPoint(pMsg,mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId(),mpCC->mg2oS_wcurmap_wclientmap)};

            if(pMP->isBad())
            {
                //could not be processed, but send weak ack
                this->SetWeakAckMP(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pMP->EstablishInitialConnections();

            if(pMP->isBad())
            {
                this->SetWeakAckMP(pMsg->mnId);
                delete pMsg;
                continue;
            }

            mpMap->AddMapPoint(pMP);
            msAcksMP.insert(pMsg->mnId);

            delete pMsg;
        }
        else if(msgpair.second.mnClientId != MAPRANGE)
        {
            ccmslam_msgs::MPred* pMsg = new ccmslam_msgs::MPred();

            *pMsg = msgpair.second;

            mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mnClientId);

            if(pMP)
            {
                //everything ok
            }
            else
            {
                if(!mpMap->IsMpDeleted(pMsg->mnId,pMsg->mnClientId))
                {
                    this->SetWeakAckMP(pMsg->mnId);
                }
            }

            if(!pMP)
            {
                delete pMsg;
                continue; //maybe it is deleted, maybe it got lost -- but it is not there
            }

            if(pMP->isBad())
            {
                delete pMsg;
                continue; //no need to process bad MPs.
            }

            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            pMP->UpdateFromMessage(pMsg,mpCC->mg2oS_wcurmap_wclientmap);

            pMP->UpdateNormalAndDepth();

            delete pMsg;
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mMpItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mnClientId << ": reached INPUT MP iteration limit" << "[" << mlBufMPin.size() << "]" << endl;
        }
        #endif
    }
}*/

void Communicator::PassKftoComm(KeyFrame* pKf, bool bIsIMUInitialized)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    if( mSensor==MONOCULAR|| (mSensor==IMU_MONOCULAR&&bIsIMUInitialized)){
        if(pKf->IsFromServer())
            return;
        mspBufferKfOut.insert(pKf);
        pKf->MarkInOutBuffer();
    }
    else{
    //    std::cout<<"NO GIBA1"<<std::endl;
    }
}

void Communicator::PassMptoComm(MapPoint* pMp, bool bIsIMUInitialized)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    if( mSensor==MONOCULAR|| (mSensor==IMU_MONOCULAR&&bIsIMUInitialized) ){
        if(pMp->IsFromServer())
            return;
        mspBufferMpOut.insert(pMp);
        pMp->MarkInOutBuffer();
    }
}
void Communicator::ErasedKfFromBuffer(KeyFrame* pKF)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    set<KeyFrame*>::iterator sit = find(mspBufferKfOut.begin(),mspBufferKfOut.end(),pKF);
    if(sit != mspBufferKfOut.end())
    {
        cout<<"EraseKfinBuffer:"<<pKF->mnId<<endl;
        mspBufferKfOut.erase(sit);
    }

    pKF->UnMarkInOutBuffer();
}

void Communicator::PassErasedMpIdtoComm(size_t mId, bool bIsIMUInitialized)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    if( mSensor==MONOCULAR|| (mSensor==IMU_MONOCULAR&&bIsIMUInitialized))
    {
        mspBufferErasedMpIdsOut.push_back(mId);
    }
}

void Communicator::PassErasedKfIdtoComm(size_t mId, bool bIsIMUInitialized)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    if( mSensor==MONOCULAR|| (mSensor==IMU_MONOCULAR&&bIsIMUInitialized))
    {
        mspBufferErasedKfIdsOut.push_back(mId);
    }
}

bool Communicator::CheckBufferKfOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferKfOut.empty());
}
/*void Communicator::DeleteMpFromBuffer(mpptr pMP) //Tongjiang: it is used in the mappoint, when the map point is bad, so at first we don't need it
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    set<mpptr>::iterator sit = find(mspBufferMpOut.begin(),mspBufferMpOut.end(),pMP);
    if(sit != mspBufferMpOut.end())
    {
        mspBufferMpOut.erase(sit);
    }
}*/

/*bool Communicator::CheckBufferKfIn()
{
    unique_lock<mutex> lock(mMutexBuffersIn);
    return(!mlBufKFin.empty());
}

bool Communicator::CheckBufferKfOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferKfOut.empty());
}

bool Communicator::CheckBufferMpIn()
{
    unique_lock<mutex> lock(mMutexBuffersIn);
    return(!mlBufMPin.empty());
}

bool Communicator::CheckBufferMpOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferMpOut.empty());
}
*/
/*void Communicator::RequestReset() //Tongjiang: used in Tracking, when initialize falied. TODO: Compare with the orb slam3 resetActive....
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
            {
                break;
            }
        }
        if(mpCC->mSysState == eSystemState::CLIENT)
            usleep(params::timings::client::miCommRate);
        else if(mpCC->mSysState == eSystemState::SERVER)
            usleep(params::timings::server::miCommRate);
        else KILLSYS
    }
}*/

/*void Communicator::ResetIfRequested()
{
    unique_lock<mutex> lockReset(mMutexReset);

    if(mbResetRequested)
    {
        if(mpCC->mSysState == eSystemState::CLIENT)
        {
            this->ResetCommunicator();
        }
        else if(mpCC->mSysState == eSystemState::SERVER)
        {
            this->ResetCommunicator();
            this->ResetMapping();
            this->ResetDatabase();
            this->ResetMap();
        }
        else
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ResetIfRequested(): invalid systems state: " << mpCC->mSysState << endl;
            throw infrastructure_ex();
        }
         mbResetRequested = false;
    }
}
*/
/*void Communicator::ResetCommunicator()
{
    unique_lock<mutex> lockIn(mMutexBuffersIn);
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    unique_lock<mutex> lock5(mMutexNearestKf);

    usleep(10000); // wait to give msg buffers time to empty

    mlBufKFin.clear();
    mlBufMPin.clear();
    mspBufferKfOut.clear();
    mspBufferMpOut.clear();

    msAcksKF.clear();
    msAcksMP.clear();

    mpNearestKF = nullptr;
    mNearestKfId = defpair;
}
*/
/*void Communicator::ResetDatabase()
{
    vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();

    for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
    {
        kfptr pKF = *vit;
        mpDatabase->erase(pKF);
    }

    mpDatabase->ResetMPs();
}
*/
/*void Communicator::ResetMapping()
{
    mpMapping->RequestReset();
}

void Communicator::ResetMap()
{
    mpMap->clear();
}

void Communicator::SetWeakAckKF(size_t id)
{
    if(mnWeakAckKF == KFRANGE)
        mnWeakAckKF = id;
    else if(id > mnWeakAckKF)
        mnWeakAckKF = id;
}

void Communicator::SetWeakAckMP(size_t id)
{
    if(mnWeakAckMP == MPRANGE)
        mnWeakAckMP = id;
    else if(id > mnWeakAckMP)
        mnWeakAckMP = id;
}

bool Communicator::kfcmp::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mId.first < pB->mId.first;
}

bool Communicator::mpcmp::operator ()(const mpptr pA, const mpptr pB) const
{
    return pA->mId.first < pB->mId.first;
}*/
} //end ns
