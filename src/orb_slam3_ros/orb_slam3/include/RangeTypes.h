#ifndef RANGETYPES_H
#define RANGETYPES_H

#include<vector>
#include<utility>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>


namespace ORB_SLAM3
{

class Range
{
public:
    Range(const float _x, const float _y, const float _z, const double &timestamp):
         x(_x), y(_y), z(_z), t(timestamp){
             d = sqrt(x*x+y*y+z*z);
         }
    Range(const float _d, const double &timestamp, const double _delta_t):
        d(_d), t(timestamp), delta_t(_delta_t){}
public:
    float x, y, z;
    float d;
    double t;
    double delta_t;
};


} //namespace ORB_SLAM3

#endif // RANGETYPES_H
