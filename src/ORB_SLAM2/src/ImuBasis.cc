#include "ImuBasis.h"

#include <iostream>

#include "GeometricFunctions.h"

using namespace std;

namespace ORB_SLAM2
{

cv::Mat NormalizeRotation(const cv::Mat &R)
{
    cv::Mat U,w,Vt;
    cv::SVDecomp(R,w,U,Vt,cv::SVD::FULL_UV);
    return U*Vt;
}

cv::Mat calcJacobianOfExponentialMap(float wvec[3])
{
    cv::Mat Jmat = cv::Mat::eye(3,3,CV_32F);

    cv::Mat hw = (cv::Mat_<float>(3,3) << 0, -wvec[2], wvec[1], 
                                        wvec[2], 0, -wvec[0],
                                        -wvec[1], wvec[0], 0);

    float d2 = wvec[0]*wvec[0] + wvec[1]*wvec[1] + wvec[2]*wvec[2];
    float d = sqrt(d2);

    if(d > 1.0e-4){
        Jmat = Jmat - ((1.0-cos(d))/d2)*hw + ((d-sin(d))/d2/d)*hw*hw;
    } 

    return Jmat.clone();
}

////
//ImuCalib
////
ImuCalib::ImuCalib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
{
    Tbc = Tbc_.clone();
    
    Tcb = cv::Mat::eye(4,4,CV_32F);
    Tcb.rowRange(0,3).colRange(0,3) = Tbc.rowRange(0,3).colRange(0,3).t();
    Tcb.rowRange(0,3).col(3) = -Tbc.rowRange(0,3).colRange(0,3).t()*Tbc.rowRange(0,3).col(3);
    
    Cov = cv::Mat::eye(6,6,CV_32F);
    const float ng2 = ng*ng;
    const float na2 = na*na;
    Cov.at<float>(0,0) = ng2;
    Cov.at<float>(1,1) = ng2;
    Cov.at<float>(2,2) = ng2;
    Cov.at<float>(3,3) = na2;
    Cov.at<float>(4,4) = na2;
    Cov.at<float>(5,5) = na2;

    CovWalk = cv::Mat::eye(6,6,CV_32F);
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;
    CovWalk.at<float>(0,0) = ngw2;
    CovWalk.at<float>(1,1) = ngw2;
    CovWalk.at<float>(2,2) = ngw2;
    CovWalk.at<float>(3,3) = naw2;
    CovWalk.at<float>(4,4) = naw2;
    CovWalk.at<float>(5,5) = naw2;
}


////
//ImuPreintegration
////
ImuPreintegration::ImuPreintegration(const ImuBias &bias, const ImuCalib &calib)
{
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(bias);
}


ImuPreintegration::ImuPreintegration(ImuPreintegration *pImuPre):
    dT(pImuPre->dT), Nga(pImuPre->Nga.clone()), NgaWalk(pImuPre->NgaWalk.clone()), mbias(pImuPre->mbias),
    deltR(pImuPre->deltR.clone()), deltV(pImuPre->deltV.clone()), deltP(pImuPre->deltP), mCov(pImuPre->mCov),
    JRbg(pImuPre->JRbg.clone()), JVba(pImuPre->JVba.clone()), JVbg(pImuPre->JVbg.clone()), JPba(pImuPre->JPba.clone()), JPbg(pImuPre->JPbg.clone())
{

}


void ImuPreintegration::Initialize(const ImuBias &bias)
{
    mbias = bias;

    deltR = cv::Mat::eye(3,3,CV_32F);
    deltV = cv::Mat::zeros(3,1,CV_32F);
    deltP = cv::Mat::zeros(3,1,CV_32F);

    JRbg = cv::Mat::zeros(3,3,CV_32F);
    JVba = cv::Mat::zeros(3,3,CV_32F);
    JVbg = cv::Mat::zeros(3,3,CV_32F);
    JPba = cv::Mat::zeros(3,3,CV_32F);
    JPbg = cv::Mat::zeros(3,3,CV_32F);

    mCov = cv::Mat::zeros(15,15,CV_32F);

    // accDir = cv::Mat::zeros(3,1,CV_32F);

    dT = 0.0;
}


void ImuPreintegration::Integrate(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &deltaT)
{

    cv::Mat acc(3,1,CV_32F);
    acc.at<float>(0) = acceleration.x - mbias.bax;
    acc.at<float>(1) = acceleration.y - mbias.bay;
    acc.at<float>(2) = acceleration.z - mbias.baz;

    cv::Mat ahat = (cv::Mat_<float>(3,3) << 0.0, -acc.at<float>(2), acc.at<float>(1),
                                            acc.at<float>(2), 0.0, -acc.at<float>(0),
                                            -acc.at<float>(1), acc.at<float>(0), 0.0);

    deltP = deltP + deltV*deltaT + 0.5*deltR*acc*deltaT*deltaT;
    deltV = deltV + deltR*acc*deltaT;

    // cout<<"A:"<<acc.at<float>(0)<<","<<acc.at<float>(1)<<","<<acc.at<float>(2)<<"  "<<deltaT<<endl;
    // cout<<"V:"<<deltV.at<float>(0)<<","<<deltV.at<float>(1)<<","<<deltV.at<float>(2)<<endl;

    cv::Mat Amat = cv::Mat::eye(9,9,CV_32F);
    cv::Mat Bmat = cv::Mat::zeros(9,6,CV_32F);

    float wbt[3], dRk[3][3];
    wbt[0] = (angVel.x - mbias.bwx)*deltaT;
    wbt[1] = (angVel.y - mbias.bwy)*deltaT;
    wbt[2] = (angVel.z - mbias.bwz)*deltaT;
    convertRodrigesToRot(wbt, dRk);
    cv::Mat Jrk = calcJacobianOfExponentialMap(wbt);

    cv::Mat dRk_cv = (cv::Mat_<float>(3,3)<<dRk[0][0], dRk[0][1], dRk[0][2],
                                            dRk[1][0], dRk[1][1], dRk[1][2],
                                            dRk[2][0], dRk[2][1], dRk[2][2]);


    JPba = JPba + JVba*deltaT - 0.5*deltaT*deltaT*deltR;
    JPbg = JPbg + JVbg*deltaT - 0.5*deltaT*deltaT*deltR*ahat*JRbg;
    JVba = JVba - deltR*deltaT;
    JVbg = JVbg - deltR*ahat*JRbg*deltaT;
    JRbg = dRk_cv.t()*JRbg - Jrk*deltaT;

    Amat.rowRange(0,3).colRange(0,3) = dRk_cv.t();
    Amat.rowRange(3,6).colRange(0,3) = -deltaT*deltR*ahat;
    Amat.rowRange(6,9).colRange(0,3) = -0.5*deltaT*deltaT*deltR*ahat;
    Amat.rowRange(6,9).colRange(3,6) = deltaT*cv::Mat::eye(3,3,CV_32F);
    Bmat.rowRange(0,3).colRange(0,3) = deltaT*Jrk;
    Bmat.rowRange(3,6).colRange(3,6) = deltaT*deltR;
    Bmat.rowRange(6,9).colRange(3,6) = 0.5*deltaT*deltaT*deltR;


    mCov.rowRange(0,9).colRange(0,9) = Amat*mCov.rowRange(0,9).colRange(0,9)*Amat.t() + Bmat*Nga*Bmat.t();
    mCov.rowRange(9,15).colRange(9,15) = mCov.rowRange(9,15).colRange(9,15) + NgaWalk;

    deltR = NormalizeRotation(deltR*dRk_cv);

    // accDir = (accDir*dT + acc*deltaT)/(dT + deltaT);

    dT += deltaT;
}


}
