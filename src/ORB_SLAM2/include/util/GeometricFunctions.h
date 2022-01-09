#ifndef GEOMETRIC_FUNCTIONS_H
#define GEOMETRIC_FUNCTIONS_H


void convertQuatToRot(float quat[4],float Rot[3][3]);

void convertRotToQuat(float Rot[3][3], float quat[4]);

//Logarithm Map
void convertRotToRodriges(float Rot[3][3], float rod[3]);

void calcJacobianOfExponentialMap(float wvec[3], float Jmat[3][3]);


//Exponential Map
void convertRodrigesToRot(float rod[3], float Rot[3][3]);

void calcDerivativeOfRotationForRodrigues(float Rod[3], float dRdr[3][3][3]);

void calcJacobianOfLogMap(float wvec[3], float invJ[3][3]);


void calcRotationFromEulerZXY(float rx, float ry, float rz, float Rot[3][3]);

void calcRotationFromEulerXYZ(float rx, float ry, float rz, float Rot[3][3]);


#endif