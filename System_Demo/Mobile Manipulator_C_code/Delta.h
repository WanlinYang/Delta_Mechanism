#ifndef DELTA_H
#define DELTA_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.1415926

/*
Parameters of Delta platform:

R1: Radius of lower platform (m)
R2: Radius of upper platform (m)
L1: Length of lower legs (m)
L2: Length of upper legs (m)
x, y, z: position of end-effector in space frame
thetalist: the array containing angles of revolute joints
*/


// calculate inverse 3x3 matrix s to get d
void Inverse3x3(const float *s, float *d);
/*
The following four functions cooperatively solve forward kinematics
*/
// Delta original equations for forward kinematics
void DeltaoFunction(float *x, float *blist, float *plist, float *Llist, float L2, float *d);
// Delta derivative equations for forward kinematics
void DeltadFunction(float *x, float *blist, float *plist, float *Llist, float L2, float *df);
// Newton-Raphson method solving equations of forward kinematics
void NewtonMethod(float *x, float *blist, float *plist, float *Llist, float L2, float *d);
// Delta forward kinematics, input and array of joint angles to get end-effector position (x,y,z)
void DeltaFkin(const float R1, const float R2, const float L1, const float L2, float *thetalist, float *p0, float *p);

// Delta inverse kinematics, input end-effector position (x,y,z) in space frame to get an array of joint angles
void DeltaIkin(const float R1, const float R2, const float L1,const float L2,const float x,const float y,const float z,float *thetalist);

// Input end-effector position (x,y,z) in space frame to get 3x3 Jacobian
void DeltaJocabian(const float R1, const float R2, const float L1,const float L2,const float x,const float y,const float z,float *J);
void DeltaForce(const float R1, const float R2, const float L1,const float L2,const float x,const float y,const float z,float *taulist,float *forcelist);

/*
Following are helper functions for matrix calculation from MR_C
*/

// Adds the n-vectors v1 and v2 to get v3.
void mr_AddVectors(const float *v1, const float *v2, const int n, float *v3);
// Substracts the n-vectors v2 from v1 to get v3.   v3 = v1-v2
void mr_MinusVectors(const float *v1, const float *v2, const int n, float *v3);
// Adds the n1 x n2 matrices s1 and s2 to get d.
void mr_AddMatrices(const float *s1, const float *s2, const int n1, const int n2, float *d);
// Substracts n1 x n2 matrices s2 frm s1 to get d.  d = s1-s2
void mr_MinusMatrices(const float *s1, const float *s2, const int n1, const int n2, float *d);
// Multiplies the n1 x n2 matrix m by the scalar s to get d.
void mr_MS(const float *m, const int n1, const int n2, const float s, float *d);
// Multiplies the n1 x n2 matrix m by n-vector v to get d.
void mr_MV(const float *m, const int n1, const int n2, const float *v, float *d);
// Multiplies n1 x n2 matrix m1 by n2 x n3  matrix m2 to get d.
void mr_MM(const float *m1, float *m2, const int n1, const int n2, const int n3, float *d);
// Copies the n-vector s to d.
void mr_CopyVector(const float *s, const int n, float *d);
// Transpose n1 x n2 matrix
void mr_Transpose(const float *s, const int n1, const int n2, float *d);


#endif