#include "GeometricFunctions.h"
#include <cmath>
#include <iostream>

using namespace std;

void convertQuatToRot(float quat[4],float Rot[3][3])
{
    for(int i=0;i<3;i++){
        Rot[i][i] = quat[0]*quat[0] + quat[i+1]*quat[i+1] - quat[(i+1)%3+1]*quat[(i+1)%3+1] - quat[(i+2)%3+1]*quat[(i+2)%3+1];
        for(int j=0;j<3;j++){
            if(j== (i+1)%3) Rot[i][j] = 2*(quat[i+1]*quat[j+1] - quat[0]*quat[4-i-j]);
            else if(j != i) Rot[i][j] = 2*(quat[i+1]*quat[j+1] + quat[0]*quat[4-i-j]);
        }
    }
}


void convertRotToQuat(float Rot[3][3], float quat[4])
{
    quat[0] = 1.0;
    quat[1] = 0.0;
    quat[2] = 0.0; 
    quat[3] = 0.0;

    float qsq[4];
    qsq[0] = (Rot[0][0] + Rot[1][1] + Rot[2][2] + 1.0)/4.0;
    
    int maxidx = 0;
    float maxsq = qsq[0];
    for(int j=0;j<3;j++){
        qsq[1+j] = (1.0+Rot[j][j]-Rot[(j+1)%3][(j+1)%3]-Rot[(j+2)%3][(j+2)%3])/4.0;
        if(qsq[1+j] > maxsq){
            maxidx = j+1;
            maxsq = qsq[1+j];
        }
    }

    if(maxidx == 0){
        quat[0] = sqrt(qsq[0]);
        quat[1] = (Rot[2][1] - Rot[1][2])/quat[0]/4.0;
        quat[2] = (Rot[0][2] - Rot[2][0])/quat[0]/4.0;
        quat[3] = (Rot[1][0] - Rot[0][1])/quat[0]/4.0;
    } else if(maxidx == 1){
        quat[1] = sqrt(qsq[1]);
        quat[0] = (Rot[2][1] - Rot[1][2])/quat[1]/4.0;
        quat[2] = (Rot[1][0] + Rot[0][1])/quat[1]/4.0;
        quat[3] = (Rot[0][2] + Rot[2][0])/quat[1]/4.0;
    } else if(maxidx == 2){
        quat[2] = sqrt(qsq[2]);
        quat[0] = (Rot[0][2] - Rot[2][0])/quat[2]/4.0;
        quat[1] = (Rot[1][0] + Rot[0][1])/quat[2]/4.0;
        quat[3] = (Rot[2][1] + Rot[1][2])/quat[2]/4.0;
    } else {
        quat[3] = sqrt(qsq[3]);
        quat[0] = (Rot[1][0] - Rot[0][1])/quat[3]/4.0;
        quat[1] = (Rot[2][0] + Rot[0][2])/quat[3]/4.0;
        quat[2] = (Rot[2][1] + Rot[1][2])/quat[3]/4.0;
    }

}


void convertRotToRodriges(float Rot[3][3], float rod[3])
{
    float theta = (Rot[0][0] + Rot[1][1] + Rot[2][2] - 1.0)/2.0;
    theta = acos(theta);

    float sinv = sin(theta);
    if(sinv != 0){
        float scale = 1.0/sinv*0.5*theta; 
        rod[0] = scale*(Rot[2][1] - Rot[1][2]);
        rod[1] = scale*(Rot[0][2] - Rot[2][0]);
        rod[2] = scale*(Rot[1][0] - Rot[0][1]);
    } else {
        rod[0] = 0.0;
        rod[1] = 0.0;
        rod[2] = 0.0;
    }
}


void calcJacobianOfExponentialMap(float wvec[3], float Jmat[3][3])
{
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            Jmat[i][j] = 0.0;
        }
        Jmat[i][i] = 1.0;
    }

    float what[3][3] = {{0.0,-wvec[2],wvec[1]},{wvec[2],0.0,-wvec[0]},{-wvec[1],wvec[0],0.0}};

    float d2 = wvec[0]*wvec[0] + wvec[1]*wvec[1] + wvec[2]*wvec[2];
    float d1 = sqrt(d2);

    if(d1 > 1.0e-4){
        float c1 = (1.0-cos(d1))/d2;
        float c2 = (d1-sin(d1))/d1/d2;
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                Jmat[i][j] += -c1*what[i][j] + c2*(what[i][0]*what[0][j]+what[i][1]*what[1][j]+what[i][2]*what[2][j]);
            }
        }
    }
}


void convertRodrigesToRot(float rod[3], float Rot[3][3])
{
    float theta = sqrt(rod[0]*rod[0] + rod[1]*rod[1] + rod[2]*rod[2]);
    float cosv = cos(theta);
    float sinv = sin(theta);

    float nrod[3];
    if(theta != 0){
        nrod[0] = rod[0]/theta;
        nrod[1] = rod[1]/theta;
        nrod[2] = rod[2]/theta;
    } else {
        nrod[0] = 1.0;
        nrod[1] = 0.0;
        nrod[2] = 0.0;
    }

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            if(i == j) Rot[i][j] = cosv;
            else if(j == (i+1)%3) Rot[i][j] = -sinv*nrod[3-i-j];
            else Rot[i][j] = sinv*nrod[3-i-j];

            Rot[i][j] += (1.0-cosv)*nrod[i]*nrod[j];
        }
    }

}


void calcDerivativeOfRotationForRodrigues(float Rod[3], float dRdr[3][3][3])
{
    float theta = sqrt(Rod[0]*Rod[0]+Rod[1]*Rod[1]+Rod[2]*Rod[2]);
    float cosv = cos(theta);
    float sinv = sin(theta);
    
    float nRod[3];
    float sintv, costv;
    if(theta != 0){
        nRod[0] = Rod[0]/theta;
        nRod[1] = Rod[1]/theta;
        nRod[2] = Rod[2]/theta;
        sintv = sinv/theta;
        costv = (1.0-cosv)/theta;
    } else {
        nRod[0] = 1.0;
        nRod[1] = 1.0;
        nRod[2] = 1.0;
        sintv = 1.0;
        costv = 0.0;
    }

    float dRdt[3][3];
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            if(i == j){
                dRdt[i][j] = -sinv;
            } else if(j == (i+1)%3){
                dRdt[i][j] = -(cosv-sintv)*nRod[3-i-j];
            } else{
                dRdt[i][j] = (cosv-sintv)*nRod[3-i-j];
            }
            dRdt[i][j] += (sinv-2.0*costv)*nRod[i]*nRod[j];
        }
    }

    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            for(int k=0;k<3;++k){
                dRdr[i][j][k] = dRdt[i][j]*nRod[k];
            }
        }
    }
    for(int i=0;i<3;++i){
        dRdr[(i+1)%3][(i+2)%3][i] += -sintv;
        dRdr[(i+2)%3][(i+1)%3][i] += sintv; 
        for(int j=0;j<3;j++){
            dRdr[i][j][i] += costv*nRod[j];
            dRdr[j][i][i] += costv*nRod[j];
        }
    }



}


void calcJacobianOfLogMap(float wvec[3], float invJ[3][3])
{
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            invJ[i][j] = 0.0;
        }
        invJ[i][i] = 1.0;
    }

    float what[3][3] = {{0.0,-wvec[2],wvec[1]},{wvec[2],0.0,-wvec[0]},{-wvec[1],wvec[0],0.0}};

    float d2 = wvec[0]*wvec[0] + wvec[1]*wvec[1] + wvec[2]*wvec[2];
    float d1 = sqrt(d2);

    if(d1 > 1.0e-4){
        float c2 = 1.0/d2 + (1.0+cos(d1))/2.0/d1/sin(d1);
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                invJ[i][j] += 0.5*what[i][j] + c2*(what[i][0]*what[0][j]+what[i][1]*what[1][j]+what[i][2]*what[2][j]);
            }
        }
    }

}


void calcRotationFromEulerZXY(float rx, float ry, float rz, float Rot[3][3])
{
    float Rx[3][3];
    float Ry[3][3];
    float Rz[3][3];
    
    Rx[0][0] = 1.0; Rx[0][1] = 0.0; Rx[0][2] = 0.0;
    Rx[1][0] = 0.0; Rx[1][1] = cos(rx); Rx[1][2] = -sin(rx);
    Rx[2][0] = 0.0; Rx[2][1] = sin(rx); Rx[2][2] = cos(rx);
    
    Ry[0][0] = cos(ry); Ry[0][1] = 0.0; Ry[0][2] = sin(ry);
    Ry[1][0] = 0.0; Ry[1][1] = 1.0; Ry[1][2] = 0.0;
    Ry[2][0] = -sin(ry); Ry[2][1] = 0.0; Ry[2][2] = cos(ry);

    Rz[0][0] = cos(rz); Rz[0][1] = -sin(rz); Rz[0][2] = 0.0;
    Rz[1][0] = sin(rz); Rz[1][1] = cos(rz); Rz[1][2] = 0.0;
    Rz[2][0] = 0.0; Rz[2][1] = 0.0; Rz[2][2] = 1.0;


    float Rxz[3][3];
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            Rxz[i][j] = 0.0;
            for(int k=0;k<3;++k){
                Rxz[i][j] += Rx[i][k]*Rz[k][j];
            }
        }
    }
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            Rot[i][j] = 0.0;
            for(int k=0;k<3;++k){
                Rot[i][j] += Ry[i][k]*Rxz[k][j];
            }
        }
    }

}


void calcRotationFromEulerXYZ(float rx, float ry, float rz, float Rot[3][3])
{
    float Rx[3][3];
    float Ry[3][3];
    float Rz[3][3];
    
    Rx[0][0] = 1.0; Rx[0][1] = 0.0; Rx[0][2] = 0.0;
    Rx[1][0] = 0.0; Rx[1][1] = cos(rx); Rx[1][2] = -sin(rx);
    Rx[2][0] = 0.0; Rx[2][1] = sin(rx); Rx[2][2] = cos(rx);
    
    Ry[0][0] = cos(ry); Ry[0][1] = 0.0; Ry[0][2] = sin(ry);
    Ry[1][0] = 0.0; Ry[1][1] = 1.0; Ry[1][2] = 0.0;
    Ry[2][0] = -sin(ry); Ry[2][1] = 0.0; Ry[2][2] = cos(ry);

    Rz[0][0] = cos(rz); Rz[0][1] = -sin(rz); Rz[0][2] = 0.0;
    Rz[1][0] = sin(rz); Rz[1][1] = cos(rz); Rz[1][2] = 0.0;
    Rz[2][0] = 0.0; Rz[2][1] = 0.0; Rz[2][2] = 1.0;

    float Ryx[3][3];
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            Ryx[i][j] = 0.0;
            for(int k=0;k<3;++k){
                Ryx[i][j] += Ry[i][k]*Rx[k][j];
            }
        }
    }
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            Rot[i][j] = 0.0;
            for(int k=0;k<3;++k){
                Rot[i][j] += Rz[i][k]*Ryx[k][j];
            }
        }
    }

}
