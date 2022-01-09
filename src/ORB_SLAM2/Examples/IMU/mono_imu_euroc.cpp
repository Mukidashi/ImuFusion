/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAccs, vector<cv::Point3f> &vGyros);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./mono_tum (Voc.bin) (Setting.yaml) (ImgPath) (TimeStamp.txt) (ImuData.csv)" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImage;
    vector<double> vTimestamp, vTimeStampImu;
    vector<cv::Point3f> vAcc, vGyro;

    LoadImages(string(argv[3]), string(argv[4]), vstrImage, vTimestamp);
    LoadIMU(string(argv[5]),vTimeStampImu, vAcc, vGyro);

    int nImages = vstrImage.size();
    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    int first_imuN = 0;
    while(vTimeStampImu[first_imuN] < vTimestamp[0]){
        first_imuN++;
    }
    first_imuN--;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true,true);

    ImuFusion *p_fusion = new ImuFusion(argv[2]);
    SLAM.SetImuFusion(p_fusion);

    thread thread_fusion(&ImuFusion::Run, p_fusion);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    int mState = SLAM.GetTrackingState();
    double overtime = 0.0;
    cv::Mat im;
    cv::Mat mTcw;
    vector<ORB_SLAM2::ImuPoint> vImuMeas;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImage[ni],CV_LOAD_IMAGE_UNCHANGED);

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImage[ni] << endl;
            return 1;
        }

        double tframe = vTimestamp[ni];
        
        if(ni < nImages - 1 && vTimestamp[ni+1] - tframe < overtime){
            overtime -= vTimestamp[ni+1] - tframe;
            continue;
        }

        vImuMeas.clear();

        if(ni > 0){
            while(vTimeStampImu[first_imuN] <= tframe){
                vImuMeas.push_back(ORB_SLAM2::ImuPoint(vAcc[first_imuN].x, vAcc[first_imuN].y, vAcc[first_imuN].z,
                                    vGyro[first_imuN].x, vGyro[first_imuN].y, vGyro[first_imuN].z, vTimeStampImu[first_imuN]));
                first_imuN++;
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        if(mState == ORB_SLAM2::Tracking::OK){
            mTcw = SLAM.TrackMonocular(im,tframe);

            if(!vImuMeas.empty()){
                for(int i=0;i<vImuMeas.size();++i){
                    p_fusion->addImuMeasure(vImuMeas[i]);
                }
            }
        } else {
            mTcw = SLAM.TrackMonocular(im, tframe, vImuMeas);
        }

        int newState = SLAM.GetTrackingState();
        if(newState == ORB_SLAM2::Tracking::OK){
            if(mState == ORB_SLAM2::Tracking::NOT_INITIALIZED){
                ORB_SLAM2::ImuPreintegration *pInitImuPre;

                //Get Preintegration from SLAM
                SLAM.GetInitImuPreintegration(pInitImuPre);
                cout<<"Init Tcw"<<endl<<mTcw<<endl;
                cout<<"dT:"<<pInitImuPre->dT<<endl;

                //Start initialize IMU
                double initTstamp = SLAM.GetInitFrameTimeStamp();
                p_fusion->startFusionInitialization(mTcw, initTstamp, tframe, pInitImuPre);

            } else {

                p_fusion->setSlamPose(mTcw, tframe);

            }

            ImuFusion::eImuFusionState fstate = p_fusion->getImuFusionState();
            if(fstate == ImuFusion::OK){
                cout<<"Enough poses are accumulated"<<endl;
                break;
            }
        }

        mState = newState;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamp[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamp[ni-1];

        if(ttrack<T){
            usleep((T-ttrack)*1e6);
        } else {
            overtime += ttrack - T;
        }
    }

    // Stop all threads
    SLAM.Shutdown();
    
    p_fusion->Shutdown();
    thread_fusion.join();


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}


void LoadIMU(const string &strPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAccs, vector<cv::Point3f> &vGyros)
{
    ifstream fImu;
    fImu.open(strPath.c_str());
    vTimeStamps.reserve(5000);
    vAccs.reserve(5000);
    vGyros.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if(s.empty() || s[0] == '#'){
            continue;
        }

        string item;
        size_t pos = 0;
        double data[7];
        int count = 0;
        while((pos = s.find(',')) != string::npos){
            item = s.substr(0, pos);
            data[count++] = stod(item);
            s.erase(0, pos+1);
        }
        item = s.substr(0, pos);
        data[6] = stod(item);

        vTimeStamps.push_back(data[0]/1e9);
        vAccs.push_back(cv::Point3f(data[4],data[5],data[6]));
        vGyros.push_back(cv::Point3f(data[1],data[2],data[3]));

    }
}