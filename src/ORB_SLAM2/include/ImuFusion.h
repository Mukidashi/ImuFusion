#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <mutex>
#include<opencv2/core/core.hpp>
#include <list>

#include "ImuBasis.h"

using namespace std;

const float GRAVITY_VALUE=9.81;

typedef struct SlamPose_struct{
    cv::Mat Tcw;
    double tstamp;
} SlamPose;


class ImuFusion{

public:
    enum eImuFusionState{
        NOT_INITIALIZED = 0,
        INITIALIZING = 1,
        OK = 2
    };

    eImuFusionState mState;

    ImuFusion(const string &strSettingsFile);

    ~ImuFusion();

    void startFusionInitialization(cv::Mat Tcw,double init_tstamp, double tstamp, ORB_SLAM2::ImuPreintegration *pImuPre);

    void addImuMeasure(const ORB_SLAM2::ImuPoint &ImuMeas);

    void setSlamPose(cv::Mat Tcw, double tstamp);

    eImuFusionState getImuFusionState();

    void Run();

    void Shutdown();

    void RequestFinish();

    bool isFinished();

    void GetImuFusionPoses(vector<cv::Mat> &vCamPoses, vector<cv::Mat> &vImuPoses);


private:

    void Initialize();

    void Fusion();

    bool CheckFinish();

    void SetFinish();

    void PreintegrateImu(double preTimeStamp, double curTimeStamp);

    void InitializeImu();

    bool LoadImuCalibration(cv::FileStorage &fSettings);

    void clearInitData();

    ORB_SLAM2::ImuCalib *mpImuCalib;
    ORB_SLAM2::ImuPreintegration *mpImuPreintegrated;

    SlamPose lastFramePose;

    std::mutex mMutexImuQueue;
    list<ORB_SLAM2::ImuPoint> mlImuQueue;

    std::mutex mMutexPoseQueue;
    list<SlamPose> mlPoseQueue;

    std::mutex mMutexInitData;
    vector<SlamPose> mvPoses;
    vector<ORB_SLAM2::ImuPreintegration *> mvImuPres; 

    std::mutex mMutexFinish;
    bool mbFinishRequested, mbFinished;

    std::mutex mMutexState;

};

#endif