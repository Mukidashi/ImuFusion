#include "ImuFusion.h"

#include <iostream>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>

#include "GeometricFunctions.h"

using namespace std;


Eigen::MatrixXf NormalizeRotation(Eigen::MatrixXf R)
{
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf nR = svd.matrixU()*svd.matrixV();
    return nR;
}


ImuFusion::ImuFusion(const string &strSettingsFile)
{
    mState = NOT_INITIALIZED;

    mbFinishRequested = false;
    mbFinished = true;

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    if(!LoadImuCalibration(fSettings)){
        cout<<"Error with IMU setting file loading"<<endl;
    }

}


ImuFusion::~ImuFusion()
{
    clearInitData();

    delete mpImuCalib;
    delete mpImuPreintegrated;
}


void ImuFusion::clearInitData()
{
    unique_lock<mutex> lock(mMutexInitData);
    
    mvPoses.clear();

    for(vector<ORB_SLAM2::ImuPreintegration *>::iterator itr=mvImuPres.begin(),iend=mvImuPres.end();itr != iend;++itr){
        ORB_SLAM2::ImuPreintegration* p_pre = *itr;
        delete p_pre;
    }
    mvImuPres.clear();

}


bool ImuFusion::LoadImuCalibration(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat Tbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty()){
        Tbc = node.mat();

        if(Tbc.rows != 4 || Tbc.cols != 4){
            b_miss_params = true;
        }
    } else {
        b_miss_params = true;
    }

    float freq, Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt()){
        freq = node.operator int();
    } else {
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal()){
        Ng = node.real();
    } else { 
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal()){
        Na = node.real();
    } else {
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal()){
        Ngw = node.real();
    } else {
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal()){
        Naw = node.real();
    } else {
        b_miss_params = true;
    }

    if(b_miss_params){
        return false;
    }


    const float sf = sqrt(freq);
    mpImuCalib = new ORB_SLAM2::ImuCalib(Tbc, Ng*sf, Na*sf, Ngw/sf, Naw/sf);
    mpImuPreintegrated = new ORB_SLAM2::ImuPreintegration(ORB_SLAM2::ImuBias(),*mpImuCalib);

    return true;
}


void ImuFusion::startFusionInitialization(cv::Mat Tcw, double init_tstamp, double tstamp, ORB_SLAM2::ImuPreintegration *pImuPre)
{

    clearInitData();

    SlamPose spose0, spose;
    spose0.Tcw = cv::Mat::eye(4,4,CV_32F);
    spose0.tstamp = init_tstamp;
    spose.Tcw = Tcw.clone();
    spose.tstamp = tstamp;

    {
        unique_lock<mutex> lock(mMutexInitData);
        mvPoses.push_back(spose0);
        mvPoses.push_back(spose);
        mvImuPres.push_back(pImuPre);
    }

    lastFramePose = spose;

    if(mpImuPreintegrated){
        delete mpImuPreintegrated;
    }
    mpImuPreintegrated = new ORB_SLAM2::ImuPreintegration(ORB_SLAM2::ImuBias(),*mpImuCalib);

    unique_lock<mutex> lock(mMutexState);
    mState = INITIALIZING;
}


void ImuFusion::addImuMeasure(const ORB_SLAM2::ImuPoint &ImuMeas)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlImuQueue.push_back(ImuMeas);
}


void ImuFusion::setSlamPose(cv::Mat Tcw, double tstamp)
{
    SlamPose spose;
    spose.Tcw = Tcw.clone();
    spose.tstamp = tstamp;
    

    unique_lock<mutex> lock(mMutexPoseQueue);
    mlPoseQueue.push_back(spose);
}


void ImuFusion::Run()
{

    eImuFusionState cur_state;

    while(1){

        if(CheckFinish()){
            break;
        }

        {
            unique_lock<mutex> lock(mMutexState);
            cur_state = mState;
        }

        if(cur_state == INITIALIZING){
            
            Initialize();
        
        } else if(cur_state == OK){

            Fusion();

        } 

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    SetFinish();
}


void ImuFusion::Initialize()
{
    //Get Frame Pose
    SlamPose spose;
    bool bNoPose = true;
    {
        unique_lock<mutex> lock(mMutexInitData);
        if(!mlPoseQueue.empty()){
            spose = mlPoseQueue.back();
            mlPoseQueue.clear();

            bNoPose = false;
        }
    }
    if(bNoPose){
        return;
    }


    //Integrate Imu
    PreintegrateImu(lastFramePose.tstamp, spose.tstamp);

    lastFramePose = spose;


    SlamPose lpose;
    {
        unique_lock<mutex> lock(mMutexInitData);
        lpose = mvPoses.back();
    }

    double tdiff = spose.tstamp - lpose.tstamp;
    if(tdiff < 0.25){
        return;
    }


    //Add Frame to Pose
    int poseN;
    ORB_SLAM2::ImuPreintegration *pImuPre = new ORB_SLAM2::ImuPreintegration(mpImuPreintegrated);
    {
        unique_lock<mutex> lock(mMutexInitData);
        mvPoses.push_back(spose);
        mvImuPres.push_back(pImuPre);
        poseN = mvPoses.size();
    }

    if(mpImuPreintegrated){
        delete mpImuPreintegrated;
    }
    mpImuPreintegrated = new ORB_SLAM2::ImuPreintegration(ORB_SLAM2::ImuBias(),*mpImuCalib);
    

    if(poseN <= 10){
        return;
    }


    //Solve Imu Parameter 
    InitializeImu();


    unique_lock<mutex> lock(mMutexState);
    mState = OK;
}


void ImuFusion::Fusion()
{
    //Get Frame Pose

    //Integrate Imu
}


void ImuFusion::PreintegrateImu(double preTimeStamp, double curTimeStamp)
{
    if(mlImuQueue.size() == 0){
        return;
    }

    vector<ORB_SLAM2::ImuPoint> vImuFromLastFrame;
    {
        unique_lock<mutex> lock(mMutexImuQueue);
        for(list<ORB_SLAM2::ImuPoint>::iterator litr = mlImuQueue.begin(), lend=mlImuQueue.end();litr != lend;){
            ORB_SLAM2::ImuPoint *p_imu = &(*litr);

            if(p_imu->t < preTimeStamp - 0.001l){
                litr = mlImuQueue.erase(litr);
                continue;
            } else if(p_imu->t < curTimeStamp - 0.001l){
                vImuFromLastFrame.push_back(*p_imu);
                litr = mlImuQueue.erase(litr);
                continue;
            } else {
                vImuFromLastFrame.push_back(*p_imu);
                break;
            }
            litr++;
        }
    }

    int imuN = vImuFromLastFrame.size() - 1;

    for(int i=0;i<imuN;++i){
        float tstep;
        cv::Point3f acc, angVel;

        if((i == 0) && (imuN > 1)){
            
            float t12 = vImuFromLastFrame[i+1].t - vImuFromLastFrame[i].t;
            float tl1 = preTimeStamp - vImuFromLastFrame[i].t;
            float ratio = tl1/t12;
            acc = (vImuFromLastFrame[i].a + vImuFromLastFrame[i+1].a + (vImuFromLastFrame[i+1].a - vImuFromLastFrame[i].a)*ratio)*0.5f;
            angVel = (vImuFromLastFrame[i].w + vImuFromLastFrame[i+1].w + (vImuFromLastFrame[i+1].w - vImuFromLastFrame[i].w)*ratio)*0.5f;
            tstep = vImuFromLastFrame[i+1].t - preTimeStamp;

        } else if(i < imuN - 1){

            acc = (vImuFromLastFrame[i].a + vImuFromLastFrame[i+1].a)*0.5f;
            angVel = (vImuFromLastFrame[i].w + vImuFromLastFrame[i+1].w)*0.5;
            tstep = vImuFromLastFrame[i+1].t - vImuFromLastFrame[i].t;
        
        } else if(i > 0 && i == imuN -1){

            float t12 = vImuFromLastFrame[i+1].t - vImuFromLastFrame[i].t;
            float tl2 = curTimeStamp - vImuFromLastFrame[i+1].t;
            float ratio = tl2/t12;
            acc = (vImuFromLastFrame[i].a + vImuFromLastFrame[i+1].a + (vImuFromLastFrame[i+1].a-vImuFromLastFrame[i].a)*ratio)*0.5f;
            angVel = (vImuFromLastFrame[i].w + vImuFromLastFrame[i+1].w + (vImuFromLastFrame[i+1].w-vImuFromLastFrame[i].w)*ratio)*0.5f;
            tstep = curTimeStamp - vImuFromLastFrame[i].t;

        } else if((i == 0) && (imuN == 1)){

            acc = vImuFromLastFrame[i].a;
            angVel = vImuFromLastFrame[i].w;
            tstep = curTimeStamp - preTimeStamp;

        }

        mpImuPreintegrated->Integrate(acc,angVel,tstep);
    }

}


void ImuFusion::InitializeImu()
{

    float lamdPriorBG = 1.0e2;
    float lamdPriorBA = 1.0e10;

    float lamd_damp = 1.0e-5;

    vector<SlamPose> vPoses;
    vector<ORB_SLAM2::ImuPreintegration*> vImuPres;
    {
        unique_lock<mutex> lock(mMutexInitData);
        for(vector<SlamPose>::iterator itr=mvPoses.begin(),iend=mvPoses.end();itr!=iend;++itr){
            SlamPose spose = *itr;
            vPoses.push_back(spose);    
        }
        for(vector<ORB_SLAM2::ImuPreintegration *>::iterator itr=mvImuPres.begin(),iend=mvImuPres.end();itr != iend;++itr){
            ORB_SLAM2::ImuPreintegration* p_pre = *itr;
            vImuPres.push_back(p_pre);
        }
    }

    //Initial Estimation
    vector<cv::Mat> vRwb, vPw, vVw;
    vector<double> vTimes, vTdelta;

    int poseN = vPoses.size();
    for(int i=0;i<poseN;++i){
        SlamPose spose = vPoses[i];
        cv::Mat Tbw = mpImuCalib->Tbc*spose.Tcw;
        cv::Mat Rbw = Tbw.rowRange(0,3).colRange(0,3);
        cv::Mat tbw = Tbw.rowRange(0,3).col(3);

        vRwb.push_back(Rbw.t());
        vPw.push_back(-Rbw.t()*tbw);
        vTimes.push_back(spose.tstamp);
    }

    for(int i=0;i<poseN-1;++i){
        double deltaT = vTimes[i+1]-vTimes[i];
        cv::Mat Vw = (vPw[i+1] - vPw[i])/deltaT;

        vVw.push_back(Vw.clone());
        if(i != 0){
            vVw[i] = vVw[i]/2.0 + Vw/2.0;
        } else {
            vVw.push_back(Vw.clone());
        }

        if(deltaT < 1.0e-4){
            printf("Time interval is too small @InitializeIMU\n");
        }

        vTdelta.push_back(deltaT);
    }

    Eigen::Matrix3f Rwg;
    cv::Mat gdirAvg = cv::Mat::zeros(3,1,CV_32F);
    for(int i=0;i<poseN-1;++i){
        ORB_SLAM2::ImuPreintegration *p_pre = vImuPres[i];

        cv::Mat gw = vRwb[i]*p_pre->deltV;
        gdirAvg -= gw;
    }
    float gleng = cv::norm(gdirAvg);
    if(gleng > 0.0){
        gdirAvg = gdirAvg/gleng;
        float axis[3];
        axis[0] = gdirAvg.at<float>(1);
        axis[1] = -gdirAvg.at<float>(0);
        axis[2] = 0.0;
        float ah = sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
        float theta = -gdirAvg.at<float>(2);
        if(theta>1.0) theta = 1.0;
        else if(theta < -1.0) theta = -1.0;
        theta = acos(theta);
        float rod[3],Rot[3][3];
        if(ah > 0.0){
            rod[0] = axis[0]/ah*theta;
            rod[1] = axis[1]/ah*theta;
            rod[2] = axis[2]/ah*theta;
        } else {
            rod[0] = 1.0;
            rod[1] = 0.0;
            rod[2] = 0.0;
        }
        convertRodrigesToRot(rod,Rot);
        for(int i=0;i<3;++i){
            Rwg(i,0) = Rot[i][0];
            Rwg(i,1) = Rot[i][1];
            Rwg(i,2) = Rot[i][2];
        }
    } else {
        Rwg = Eigen::Matrix3f::Identity();
    }

    float Scale = 1.0;
    float delBias[6];
    for(int i=0;i<6;++i){
        delBias[i] = 0.0;
    }

    cout<<gdirAvg<<endl;
    cout<<Rwg<<endl;

    //Information Matrix
    vector<Eigen::MatrixXf> vImat;
    for(int i=0;i<poseN-1;++i){
        ORB_SLAM2::ImuPreintegration *p_pre = vImuPres[i];
        
        cv::Mat cvImat = p_pre->mCov.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
        Eigen::MatrixXf Imat(9,9);
        for(int j=0;j<9;++j){
            for(int k=0;k<9;++k){
                Imat(j,k) = cvImat.at<float>(j,k);
            }
        }
        Imat = (Imat+Imat.transpose())/2.0;
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float,9,9>> es(Imat);
        Eigen::Matrix<float,9,1> eigs = es.eigenvalues();
        for(int j=0;j<9;++j){
            if(eigs[j] < 1.0e-12){
                eigs[i] = 0.0;
            }
        }
        Imat = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
        
        vImat.push_back(Imat);

    }


    //Process
    int varN = 9 + 3*poseN;
    int iterN = 0;
    int maxIterN = 50;
    while(1){

        float gdirw[3];
        gdirw[0] = -GRAVITY_VALUE*Rwg(0,2);
        gdirw[1] = -GRAVITY_VALUE*Rwg(1,2);
        gdirw[2] = -GRAVITY_VALUE*Rwg(2,2);

        Eigen::SparseMatrix<float> Amat(varN, varN);
        Eigen::VectorXf bvec = Eigen::VectorXf::Zero(varN);
        vector<Eigen::Triplet<float>> vCTrip;

        //for check
        float rdiff_avg = 0.0;
        float vdiff_avg = 0.0;
        float pdiff_avg = 0.0;

        float cEval = 0.0;
        for(int i=0;i<poseN-1;++i){
            Eigen::MatrixXf gradr = Eigen::MatrixXf::Zero(9,15);

            ORB_SLAM2::ImuPreintegration *p_pre = vImuPres[i];
            cv::Mat Ri = vRwb[i];
            cv::Mat Rj = vRwb[i+1];
            cv::Mat Rij = Ri.t()*Rj;
            cv::Mat IRijtRij = p_pre->deltR.t()*Rij;
            float dRdbgRod[3],dRdbgRot[3][3],rRexp[3][3];
            for(int j=0;j<3;++j){
                dRdbgRod[j] = p_pre->JRbg.at<float>(j,0)*delBias[0] + p_pre->JRbg.at<float>(j,1)*delBias[1] + 
                               p_pre->JRbg.at<float>(j,2)*delBias[2];
            }
            convertRodrigesToRot(dRdbgRod,dRdbgRot);
            for(int j=0;j<3;++j){
                for(int k=0;k<3;++k){
                    rRexp[j][k] = dRdbgRot[0][j]*IRijtRij.at<float>(0,k) + dRdbgRot[1][j]*IRijtRij.at<float>(1,k) + 
                                   dRdbgRot[2][j]*IRijtRij.at<float>(2,k); 
                }
            }
            float rR[3];
            convertRotToRodriges(rRexp,rR);
            
            //Bias
            float Jacobr[3][3],invJacor[3][3];
            float JrJR[3][3], EtJrJR[3][3];
            calcJacobianOfLogMap(rR,invJacor);
            calcJacobianOfExponentialMap(dRdbgRod,Jacobr);
            for(int j=0;j<3;++j){
                for(int k=0;k<3;++k){
                    JrJR[j][k] = 0.0;
                    for(int l=0;l<3;++l){
                        JrJR[j][k] += Jacobr[j][l]*p_pre->JRbg.at<float>(l,k);
                    }
                }
            }
            for(int j=0;j<3;++j){
                for(int k=0;k<3;++k){
                    EtJrJR[j][k] = 0.0;
                    for(int l=0;l<3;++l){
                        EtJrJR[j][k] += rRexp[l][j]*JrJR[l][k];
                    }
                }
            }
            for(int j=0;j<3;++j){
                for(int k=0;k<3;++k){
                    for(int l=0;l<3;++l){
                        gradr(j,3+k) -= invJacor[j][l]*EtJrJR[l][k];
                    }
                }
            }


            float RtV[3], intVij[3], rV[3];
            float RtP[3], intPij[3], rP[3];
            float dTi = (float)vTdelta[i];
            for(int j=0;j<3;++j){
                RtV[j] = 0.0;
                intVij[j] = p_pre->deltV.at<float>(j);
                RtP[j] = 0.0;
                intPij[j] = p_pre->deltP.at<float>(j);
                for(int k=0;k<3;++k){
                    RtV[j] += Ri.at<float>(k,j)*(Scale*(vVw[i+1].at<float>(k) - vVw[i].at<float>(k)) - gdirw[k]*dTi);
                    intVij[j] += p_pre->JVbg.at<float>(j,k)*delBias[k] + p_pre->JVba.at<float>(j,k)*delBias[3+k];
                    RtP[j] += Ri.at<float>(k,j)*(Scale*(vPw[i+1].at<float>(k) - vPw[i].at<float>(k) - vVw[i].at<float>(k)*dTi) - 0.5*gdirw[k]*dTi*dTi);
                    intPij[j] += p_pre->JPbg.at<float>(j,k)*delBias[k] + p_pre->JPba.at<float>(j,k)*delBias[3+k];

                    //Scale + Rwg
                    gradr(3+j,0) += Ri.at<float>(k,j)*Scale*(vVw[i+1].at<float>(k) - vVw[i].at<float>(k));
                    gradr(6+j,0) += Ri.at<float>(k,j)*Scale*(vPw[i+1].at<float>(k) - vPw[i].at<float>(k) - vVw[i].at<float>(k)*dTi);
                    gradr(3+j,1) -= Ri.at<float>(k,j)*GRAVITY_VALUE*Rwg(k,1)*dTi;
                    gradr(3+j,2) -= -Ri.at<float>(k,j)*GRAVITY_VALUE*Rwg(k,0)*dTi;
                    gradr(6+j,1) -= Ri.at<float>(k,j)*0.5*GRAVITY_VALUE*Rwg(k,1)*dTi*dTi;
                    gradr(6+j,2) -= -Ri.at<float>(k,j)*0.6*GRAVITY_VALUE*Rwg(k,0)*dTi*dTi;
                    // Bias
                    gradr(3+j,3+k) -= p_pre->JVbg.at<float>(j,k);
                    gradr(3+j,6+k) -= p_pre->JVba.at<float>(j,k);
                    gradr(6+j,3+k) -= p_pre->JPbg.at<float>(j,k);
                    gradr(6+j,6+k) -= p_pre->JPba.at<float>(j,k);
                    // Veloc
                    gradr(3+j,9+k) -= Ri.at<float>(k,j);
                    gradr(3+j,12+k) += Ri.at<float>(k,j);
                    gradr(6+j,9+k) -= Ri.at<float>(k,j)*dTi;
                }
                rV[j] = RtV[j] - intVij[j];
                rP[j] = RtP[j] - intPij[j];
            }

            Eigen::MatrixXf InfoMat = vImat[i];
            for(int j=0;j<3;++j){
                for(int k=0;k<3;++k){
                    cEval += InfoMat(j,k)*rR[j]*rR[k];
                    cEval += InfoMat(3+j,3+k)*rV[j]*rV[k];
                    cEval += InfoMat(6+j,6+k)*rP[j]*rP[k];
                    cEval += InfoMat(j,3+k)*rR[j]*rV[k];
                    cEval += InfoMat(3+j,k)*rV[j]*rR[k];
                    cEval += InfoMat(j,6+k)*rR[j]*rP[k];
                    cEval += InfoMat(6+j,k)*rP[j]*rR[k];
                    cEval += InfoMat(3+j,6+k)*rV[j]*rP[k];
                    cEval += InfoMat(6+j,3+k)*rP[j]*rV[k];
                }
                for(int k=0;k<9;++k){
                    for(int l=0;l<9;++l){
                        bvec(l) -= InfoMat(j,k)*rR[j]*gradr(k,l);
                        bvec(l) -= InfoMat(3+j,k)*rV[j]*gradr(k,l);
                        bvec(l) -= InfoMat(6+j,k)*rP[j]*gradr(k,l);    
                    }
                    for(int l=0;l<3;++l){
                        bvec(9+3*i+l) -= InfoMat(j,k)*rR[j]*gradr(k,9+l);
                        bvec(9+3*i+l) -= InfoMat(3+j,k)*rV[j]*gradr(k,9+l);
                        bvec(9+3*i+l) -= InfoMat(6+j,k)*rP[j]*gradr(k,9+l);
                        bvec(12+3*i+l) -= InfoMat(j,k)*rR[j]*gradr(k,12+l);
                        bvec(12+3*i+l) -= InfoMat(3+j,k)*rV[j]*gradr(k,12+l);
                        bvec(12+3*i+l) -= InfoMat(6+j,k)*rP[j]*gradr(k,12+l);
                    }
                }
            }

            Eigen::MatrixXf Ai = gradr.transpose()*InfoMat*gradr;
            for(int j=0;j<9;++j){
                for(int k=0;k<9;++k){
                    vCTrip.push_back(Eigen::Triplet<float>(j,k,Ai(j,k)));
                }
                for(int k=0;k<6;++k){
                    vCTrip.push_back(Eigen::Triplet<float>(j,9+3*i+k, Ai(j,9+k)));
                    vCTrip.push_back(Eigen::Triplet<float>(9+3*i+k,j, Ai(9+k,j)));
                }
            }
            for(int j=0;j<6;++j){
                for(int k=0;k<6;++k){
                    vCTrip.push_back(Eigen::Triplet<float>(9+3*i+j,9+3*i+k,Ai(9+j,9+k)));
                }
            }

            rdiff_avg += sqrt(rR[0]*rR[0]+rR[1]*rR[1]+rR[2]*rR[2]);
            vdiff_avg += sqrt(rV[0]*rV[0]+rV[1]*rV[1]+rV[2]*rV[2]);
            pdiff_avg += sqrt(rP[0]*rP[0]+rP[1]*rP[1]+rP[2]*rP[2]);

        }

        cEval += lamdPriorBG*(delBias[0]*delBias[0] + delBias[1]*delBias[1] + delBias[2]*delBias[2]);
        cEval += lamdPriorBA*(delBias[3]*delBias[3] + delBias[4]*delBias[4] + delBias[5]*delBias[5]);

        for(int i=0;i<3;++i){
            bvec(3+i) -= lamdPriorBG*delBias[i];
            bvec(6+i) -= lamdPriorBA*delBias[3+i];
            vCTrip.push_back(Eigen::Triplet<float>(3+i,3+i,lamdPriorBG));
            vCTrip.push_back(Eigen::Triplet<float>(6+i,6+i,lamdPriorBA));
        }

        rdiff_avg /= (float)(poseN-1);
        vdiff_avg /= (float)(poseN-1);
        pdiff_avg /= (float)(poseN-1);
        cout<<"Iter:"<<iterN<<" Eval:"<<cEval<<" dR:"<<rdiff_avg/M_PI*180.0<<" dV:"<<vdiff_avg<<" dP:"<<pdiff_avg<<endl;


        Amat.setFromTriplets(vCTrip.begin(),vCTrip.end());

        //Solve
        Eigen::VectorXf sol;
        bool CholeskyFailed = false;
        float prev_lamd_damp = 0.0;
        while(1){

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> ldlt(Amat);
            sol = ldlt.solve(bvec);

            Eigen::VectorXf rvec = Amat*sol - bvec;
            float rh = rvec.norm();
            float bh = bvec.norm();

            // cout<<" Damp:"<<lamd_damp<<" "<<rh<<" ("<<bh<<" "<<rh/bh<<")"<<endl;
            if(rh < bh*1.0e-4){
                break;
            }

            float addVal = lamd_damp - prev_lamd_damp;
            for(int i=0;i<varN;++i){
                Amat.coeffRef(i,i) += addVal;
            }

            prev_lamd_damp = lamd_damp; 
            lamd_damp *= 10.0;
            if(lamd_damp > 1.0e2){
                cout<<"Cholesky Decmp Failed"<<endl;
                break;
            }
        }
        if(CholeskyFailed){
            break;
        }
        if(lamd_damp > 1.0e-5){
            lamd_damp /= 10.0;
        }

        //Step Size
        float nEval;
        float alpha = 1.0;
        bool bNotUpdated = false;
        while(1){
            float nScale = Scale*exp(alpha*sol(0));
            Eigen::Matrix3f nRwg;
            float nrod[3],drwg[3][3];
            nrod[0] = alpha*sol(1);
            nrod[1] = alpha*sol(2);
            nrod[2] = 0.0;
            convertRodrigesToRot(nrod,drwg);
            for(int i=0;i<3;++i){
                for(int j=0;j<3;++j){
                    nRwg(i,j) = Rwg(i,0)*drwg[0][j] + Rwg(i,1)*drwg[1][j] + Rwg(i,2)*drwg[2][j];
                }
            }
            nRwg = NormalizeRotation(nRwg);
            float ndelBias[6];
            for(int i=0;i<6;++i){
                ndelBias[i] = delBias[i] + alpha*sol(3+i);
            }

            float ngdir[3];
            ngdir[0] = -GRAVITY_VALUE*nRwg(0,2);
            ngdir[1] = -GRAVITY_VALUE*nRwg(1,2);
            ngdir[2] = -GRAVITY_VALUE*nRwg(2,2);

            nEval = 0.0;
            for(int i=0;i<poseN-1;++i){
                ORB_SLAM2::ImuPreintegration *p_pre = vImuPres[i];
                cv::Mat Ri = vRwb[i];
                cv::Mat Rj = vRwb[i+1];
                cv::Mat Rij = Ri.t()*Rj;
                cv::Mat IRijtRij = p_pre->deltR.t()*Rij;
                float dRdbgRod[3], dRdbgRot[3][3], rRexp[3][3];
                for(int j=0;j<3;++j){
                    dRdbgRod[j] = p_pre->JRbg.at<float>(j,0)*ndelBias[0] + p_pre->JRbg.at<float>(j,1)*ndelBias[1] +
                                   p_pre->JRbg.at<float>(j,2)*ndelBias[2];
                }
                convertRodrigesToRot(dRdbgRod, dRdbgRot);
                for(int j=0;j<3;++j){
                    for(int k=0;k<3;++k){
                        rRexp[j][k] = dRdbgRot[0][j]*IRijtRij.at<float>(0,k) + dRdbgRot[1][j]*IRijtRij.at<float>(1,k) + dRdbgRot[2][j]*IRijtRij.at<float>(2,k);
                    }
                }
                float rR[3];
                convertRotToRodriges(rRexp,rR);

                float RtV[3], intVij[3], rV[3];
                float RtP[3], intPij[3], rP[3];
                float dTi = (float)vTdelta[i];
                Eigen::Vector3f nVi,nVj;
                for(int j=0;j<3;++j){
                    nVi(j) = vVw[i].at<float>(j) + alpha*sol(9+3*i+j);
                    nVj(j) = vVw[i+1].at<float>(j) + alpha*sol(12+3*i+j);
                }
                for(int j=0;j<3;++j){
                    RtV[j] = 0.0;
                    intVij[j] = p_pre->deltV.at<float>(j);
                    RtP[j] = 0.0;
                    intPij[j] = p_pre->deltP.at<float>(j);
                    for(int k=0;k<3;++k){
                        RtV[j] += Ri.at<float>(k,j)*(nScale*(nVj(k) - nVi(k)) - ngdir[k]*dTi);
                        intVij[j] += p_pre->JVbg.at<float>(j,k)*ndelBias[k] + p_pre->JVba.at<float>(j,k)*ndelBias[3+k];
                        RtP[j] += Ri.at<float>(k,j)*(nScale*(vPw[i+1].at<float>(k) - vPw[i].at<float>(k) - nVi(k)*dTi) - 0.5*ngdir[k]*dTi*dTi);
                        intPij[j] += p_pre->JPbg.at<float>(j,k)*ndelBias[k] + p_pre->JPba.at<float>(j,k)*ndelBias[3+k];
                    }
                    rV[j] = RtV[j] - intVij[j];
                    rP[j] = RtP[j] - intPij[j];
                }

                Eigen::MatrixXf InfoMat = vImat[i];
                for(int j=0;j<3;++j){
                    for(int k=0;k<3;++k){
                        nEval += InfoMat(j,k)*rR[j]*rR[k];
                        nEval += InfoMat(3+j,3+k)*rV[j]*rV[k];
                        nEval += InfoMat(6+j,6+k)*rP[j]*rP[k];
                        nEval += InfoMat(j,3+k)*rR[j]*rV[k];
                        nEval += InfoMat(3+j,k)*rV[j]*rR[k];
                        nEval += InfoMat(j,6+k)*rR[j]*rP[k];
                        nEval += InfoMat(6+j,k)*rP[j]*rR[k];
                        nEval += InfoMat(3+j,6+k)*rV[j]*rP[k];
                        nEval += InfoMat(6+j,3+k)*rP[j]*rV[k];
                    }
                }
            }

            nEval += lamdPriorBG*(ndelBias[0]*ndelBias[0]+ndelBias[1]*ndelBias[1]+ndelBias[2]*ndelBias[2]);
            nEval += lamdPriorBA*(ndelBias[3]*ndelBias[3]+ndelBias[4]*ndelBias[4]+ndelBias[5]*ndelBias[5]);
            
            if(nEval < cEval){
                cout<<"  alpha:"<<alpha<<" Eval:"<<nEval<<endl;

                //update
                Scale = nScale;
                for(int i=0;i<3;++i){
                    for(int j=0;j<3;++j){
                        Rwg(i,j) = nRwg(i,j);
                    }
                }
                for(int i=0;i<6;++i){
                    delBias[i] = ndelBias[i];
                }
                for(int i=0;i<poseN;++i){
                    for(int j=0;j<3;++j){
                        vVw[i].at<float>(j) = vVw[i].at<float>(j) + alpha*sol(9+3*i+j);
                    }
                }
                break;
            }

            alpha *= 0.5;
            if(alpha <1.0e-4){
                bNotUpdated = true;
                break;
            }
        }

        if(bNotUpdated){
            cout<<"Step Size Search Failed"<<endl;
            break;
        }


        //Convergence Check
        if(cEval - nEval < cEval*1.0e-3){
            cout<<"Imu Init Optimization Converge!!"<<endl;
            break;
        }


        if(iterN >= maxIterN) break;
        iterN++;
    }


    //Set Optimized Values
    cout<<"Scale:"<<Scale<<" bias:"<<delBias[0]<<" "<<delBias[1]<<" "<<delBias[2]<<" "<<delBias[3]
        <<" "<<delBias[4]<<" "<<delBias[5]<<endl;

}


ImuFusion::eImuFusionState ImuFusion::getImuFusionState()
{
    unique_lock<mutex> lock(mMutexState);
    return mState;
}


void ImuFusion::GetImuFusionPoses(vector<cv::Mat> &vCamPoses, vector<cv::Mat> &vImuPoses)
{

    vector<cv::Mat> vdR, vdP;
    {
        unique_lock<mutex> lock(mMutexInitData);
        for(vector<SlamPose>::iterator itr=mvPoses.begin(),iend=mvPoses.end();itr != iend;++itr){
            SlamPose spose = *itr;
            vCamPoses.push_back(spose.Tcw.clone());
        }

        for(vector<ORB_SLAM2::ImuPreintegration *>::iterator itr=mvImuPres.begin(),iend=mvImuPres.end();itr != iend;++itr){
            ORB_SLAM2::ImuPreintegration *p_pre = *itr;
            vdR.push_back(p_pre->deltR.clone());
            vdP.push_back(p_pre->deltP.clone());
        }
    }

    for(int i=0;i<vdR.size();++i){
        cv::Mat imuPose = cv::Mat::eye(4,4,CV_32F);
        imuPose.rowRange(0,3).colRange(0,3) = vdR[i];
        imuPose.rowRange(0,3).col(3) = vdP[i];
    }

}


void ImuFusion::Shutdown()
{
    RequestFinish();

    while(!isFinished()){
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
}


void ImuFusion::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}


bool ImuFusion::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


bool ImuFusion::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}


void ImuFusion::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}
