#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <vector>
#include <thread>
#include <mutex>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <numeric>

#define MAPRANGE std::numeric_limits<uint8_t>::max()
#define KFRANGE std::numeric_limits<uint16_t>::max()
#define MPRANGE std::numeric_limits<uint32_t>::max()
#define GREEN   "\033[32m"      /* Green */
#define RED   "\033[31m"      /* Green */
#define RST  "\033[0m"

namespace ORB_SLAM3{
using namespace std;

typedef pair<uint8_t, size_t> idpair;

enum eSystemState{
    NOTYPE=-1,
    CLIENT=0,
    SERVER=1
};

enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4
};

struct ORBParameters{
    // general parameters for the ORB detector
    int maxFrames, nFeatures, nLevels, iniThFAST, minThFAST;
    bool RGB;
    float scaleFactor, depthMapFactor, thDepth;
    // camera parameters
    string camera_type;
    float fx, fy, cx, cy, baseline, camera_bf;
    float k1, k2, p1, p2, k3, k4;
    // imu Parameters
    cv::Mat Tbc;
    float freq, Ng, Na, Ngw, Naw;

};

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};



class infrastructure_ex : public std::exception
{
public:
    virtual const char* what() const throw()
      {
        return "EXCEPTION: Bad Infrastructure";
      }
};

class UniqueIdDispenser
{
public:
    UniqueIdDispenser() : mLastId(0) {}

    size_t GetId()
    {
        unique_lock<mutex> lock(mMutexId);
        ++mLastId;

        return mLastId;
    }

    size_t GetLastId()
    {
        unique_lock<mutex> lock(mMutexId);
        return mLastId;
    }

    void SetLastId(size_t id)
    {
        mLastId = id;
    }

private:
    size_t mLastId;
    mutex mMutexId;
};

} //end namespace

#endif
