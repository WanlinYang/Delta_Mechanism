#ifndef DELTA_H
#define DELTA_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159

// calculate inverse 3x3 matrix
void Inverse3x3(const double *s, double *d);

// Delta original equations for forward kinematics
void DeltaoFunction(double *x, double *blist, double *plist, double *Llist, double L2, double *d);

// Delta derivative equations for forward kinematics
void DeltadFunction(double *x, double *blist, double *plist, double *Llist, double L2, double *df);

// Newton-Raphson method solving equations of forward kinematics
void NewtonMethod(double *x, double *blist, double *plist, double *Llist, double L2, double *d);

// Delta forward kinematics
void DeltaFkin(const double R1, const double R2, const double L1, const double L2, double *thetalist, double *p0, double *p);

// Delta inverse kinematics
void DeltaIkin(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *thetalist);

// Delta Jacobian
void DeltaJocabian(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *J);


// Following are helper functions for matrix calculation


void mr_Print(const char *str);
void mr_PrintMatrix(double *m, const int n1, const int n2);
void mr_PrintVector(const double *v, const int n);
void mr_AddVectors(const double *v1, const double *v2, const int n, double *v3);
void mr_MinusVectors(const double *v1, const double *v2, const int n, double *v3);
void mr_AddMatrices(const double *s1, const double *s2, const int n1, const int n2, double *d);
void mr_MinusMatrices(const double *s1, const double *s2, const int n1, const int n2, double *d);
void mr_MS(const double *m, const int n1, const int n2, const double s, double *d);
void mr_MV(const double *m, const int n1, const int n2, const double *v, double *d);
void mr_MM(const double *m1, double *m2, const int n1, const int n2, const int n3, double *d);
void mr_CopyVector(const double *s, const int n, double *d);



#endif