    bool bSentOnce

    uint16 mnId # unique KF id
    uint16 mnOriginMapId
    uint8 mnClientId
    uint32 mnUniqueId
    float64 dTimestamp
    bool mbAck

#Grid (to speed up feature matching)
    int16 mnGridCols
    int16 mnGridRows
    float32 mfGridElementWidthInv
    float32 mfGridElementHeightInv

#Calibration parameters
    float32 fx
    float32 fy
    float32 cx
    float32 cy
    float32 invfx
    float32 invfy

#Number of KeyPoints
    int16 N

#KeyPoints, stereo coordinate and descriptors (all associated by an index)
    CvKeyPoint[] mvKeysUn
    Descriptor[] mDescriptors

#Scale
    int8 mnScaleLevels
    float32 mfScaleFactor
    float32 mfLogScaleFactor
    float32[8] mvScaleFactors
    float32[8] mvLevelSigma2
    float32[8] mvInvLevelSigma2

#Image bounds and calibration
    int16 mnMinX
    int16 mnMinY
    int16 mnMaxX
    int16 mnMaxY
    float32[9] mK

#REFERENCE KFS
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

#MapPoints associated to keypoints
    uint32[] mvpMapPoints_Ids
    uint8[] mvpMapPoints_ClientIds
    uint16[] mvpMapPoints_VectId

    bool mbPoseChanged

#Bad flags
    bool mbBad

#if the kf is the init
    bool mbIsInit
    bool mbIsVirtualInit

#for imu
    PreintegratedIMU mImuPreintegrated
    float32[16] mTcprev
    float32[3] mVcp
    float32[3] mGb
    float32[3] mAb
    uint16 mpPrevKfId
    uint8 mpPrevKfClientId
    bool mbIsGIBA1
    bool mbIsGIBA2
