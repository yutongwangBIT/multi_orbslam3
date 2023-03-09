uint32 mnId # unique MP id
uint16 mnOriginMapId
uint8 mnClientId # client id
#uint32 mnUniqueId
bool mbAck

#Position in absolute coordinates
     float32[3] mPosPred
     float32[3] mPosPar

     bool mbNormalAndDepthChanged

    bool mbServerBA
float32[3] mWorldPos

#Reference KeyFrame
     uint16 mpPredKFId
     uint8 mpPredKFClientId
     uint16 mpParKFId
     uint8 mpParKFClientId

#Bad flag
     bool mbBad

#Visualization
     bool mbMultiUse
