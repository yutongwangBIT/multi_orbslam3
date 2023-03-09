    Header header
    uint32 mMsgId

    uint16 mnId
    KF[] Keyframes
    KFred[] KFUpdates
    MP[] MapPoints
    MPred[] MPUpdates

#bool bAck
#uint32 Ack #should not be necessary any more
#uint16[] vAckKFs
#uint32[] vAckMPs
#uint16 WeakAckKF
#uint32 WeakAckMP

    uint16[] vErasedKFs
    uint16[] vErasedMPs

    uint16 ClosestKf_Id
    uint8 ClosestKf_ClientId

    float32 mScale
    float32[9] mRgw
