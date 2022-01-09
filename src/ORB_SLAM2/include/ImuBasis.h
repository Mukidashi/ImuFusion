#ifndef IMU_BASIS_H
#define IMU_BASIS_H


#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class ImuPoint{

public:
    ImuPoint(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, double timestamp):
             a(a_x, a_y, a_z), w(w_x, w_y, w_z), t(timestamp){}
    cv::Point3f a, w;
    double t;
};


class ImuBias
{
public:
    ImuBias(): bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0){}

    ImuBias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z, 
         const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
         bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){}


    float bax, bay, baz;
    float bwx, bwy, bwz;
};


class ImuCalib
{
public:
    ImuCalib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw);

    cv::Mat Tcb;
    cv::Mat Tbc;
    cv::Mat Cov, CovWalk;
};


class ImuPreintegration{

public:
    ImuPreintegration(const ImuBias &bias, const ImuCalib &calib);

    ImuPreintegration(ImuPreintegration* pImuPre);

    void Initialize(const ImuBias &bias);

    void Integrate(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &deltaT);

    ImuBias mbias;

    float dT;
    cv::Mat deltR, deltV, deltP;

    cv::Mat Nga, NgaWalk;
    cv::Mat mCov;
    cv::Mat JRbg, JVba, JVbg, JPba, JPbg;

    // cv::Mat accDir;


private:

};

}

#endif