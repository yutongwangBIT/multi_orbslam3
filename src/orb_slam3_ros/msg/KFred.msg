    uint16 mnId # unique KF id
    uint16 mnOriginMapId
    uint8 mnClientId # client id
    uint32 mnUniqueId
    bool mbAck

#Poses
    float32[16] mTcw
    float32[16] mTcpred
    float32[16] mTcpredpred
    float32[16] mTcpar

    uint16 mpPred_KfId
    uint8 mpPred_KfClientId
    uint16 mpPredPred_KfId
    uint8 mpPredPred_KfClientId

    uint16 mpPar_KfId
    uint8 mpPar_KfClientId

#bool mbServerBA

#Bad flags
    bool mbBad

#for imu
    PreintegratedIMU mImuPreintegrated
    float32[16] mTcprev
    float32[3] mVcp
    float32[3] mGb
    float32[3] mAb
    uint16 mpPrevKfId
    uint8 mpPrevKfClientId

    bool mbPoseLock

#ADDED FOR SERVER TO CLIENT UPDATE
    uint32[] mvpMapPoints_Ids
    uint8[] mvpMapPoints_ClientIds
    uint16[] mvpMapPoints_VectId
