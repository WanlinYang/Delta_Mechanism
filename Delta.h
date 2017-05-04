#ifndef DELTA_H
#define DELTA_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159

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
void Inverse3x3(const double *s, double *d);

/*
Following four functions cooperatively solve forward kinematics
*/
// Delta original equations for forward kinematics
void DeltaoFunction(double *x, double *blist, double *plist, double *Llist, double L2, double *d);
// Delta derivative equations for forward kinematics
void DeltadFunction(double *x, double *blist, double *plist, double *Llist, double L2, double *df);
// Newton-Raphson method solving equations of forward kinematics
void NewtonMethod(double *x, double *blist, double *plist, double *Llist, double L2, double *d);
// Delta forward kinematics, input and array of joint angles to get end-effector position (x,y,z)
void DeltaFkin(const double R1, const double R2, const double L1, const double L2, double *thetalist, double *p0, double *p);

// Delta inverse kinematics, input end-effector position (x,y,z) in space frame to get an array of joint angles
void DeltaIkin(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *thetalist);

// Input end-effector position (x,y,z) in space frame to get 3x3 Jacobian
void DeltaJocabian(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *J);

/*
Following are helper functions for matrix calculation from MR_C
*/

// User should modify source for mr_Print as appropriate for their system.
void mr_Print(const char *str);
// Prints the n1 x n2 matrix m.
void mr_PrintMatrix(double *m, const int n1, const int n2);

// Prints the n-vector v.
void mr_PrintVector(const double *v, const int n);

// Adds the n-vectors v1 and v2 to get v3.
void mr_AddVectors(const double *v1, const double *v2, const int n, double *v3);

// Minus the n-vectors v2 from v1 to get v3.   v3 = v1-v2
void mr_MinusVectors(const double *v1, const double *v2, const int n, double *v3);

// Adds the n1 x n2 matrices s1 and s2 to get d.
void mr_AddMatrices(const double *s1, const double *s2, const int n1, const int n2, double *d);

// Minus n1 x n2 matrices s2 frm s1 to get d.  d = s1-s2
void mr_MinusMatrices(const double *s1, const double *s2, const int n1, const int n2, double *d);

// Multiplies the n1 x n2 matrix m by the scalar s to get d.
void mr_MS(const double *m, const int n1, const int n2, const double s, double *d);

// Multiplies the n1 x n2 matrix m by n-vector v to get d.
void mr_MV(const double *m, const int n1, const int n2, const double *v, double *d);

// Multiplies n1 x n2 matrix m1 by n2 x n3  matrix m2 to get d.
void mr_MM(const double *m1, double *m2, const int n1, const int n2, const int n3, double *d);

// Copies the n-vector s to d.
void mr_CopyVector(const double *s, const int n, double *d);



#endif