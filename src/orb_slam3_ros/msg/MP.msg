    bool mbSentOnce

    uint32 mnId # unique MP id
    uint16 mnOriginMapId
    uint8 mnClientId # client id
    uint32 mnUniqueId
    int16 mnFirstKFid

#Position in absolute coordinates
     float32[3] mPosPred
     float32[3] mPosPar
     bool mbPoseChanged

     #bool mbServerBA
     float32[3] mWorldPos

#Keyframes observing the point and associated index in keyframe
     uint16[] mObservations_KFIDs
     uint8[] mObservations_KFClientIDs
     uint16[] mObservations_n

     RefPosKF[] mObservations_KFs_RefPose

#Mean viewing direction
     float32[3] mNormalVector
     #bool mbNormalAndDepthChanged

#Best descriptor to fast matching
     uint8[32] mDescriptor

#Reference KeyFrame
     uint16 mpPredKFId
     uint8 mpPredKFClientId
     uint16 mpParKFId
     uint8 mpParKFClientId
     uint16 mpRefKFId

#Bad flag
     bool mbBad

#Scale invariance distances
     float32 mfMinDistance
     float32 mfMaxDistance

#Visualization
     #bool mbMultiUse
