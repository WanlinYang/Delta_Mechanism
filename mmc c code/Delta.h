#ifndef DELTA_H
#define DELTA_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.1415926

void Inverse3x3(const float *s, float *d);
void DeltaoFunction(float *x, float *blist, float *plist, float *Llist, float L2, float *d);
void DeltadFunction(float *x, float *blist, float *plist, float *Llist, float L2, float *df);
void NewtonMethod(float *x, float *blist, float *plist, float *Llist, float L2, float *d);
void DeltaFkin(const float R1, const float R2, const float L1, const float L2, float *thetalist, float *p0, float *p);
void DeltaIkin(const float R1, const float R2, const float L1,const float L2,const float x,const float y,const float z,float *thetalist);
void DeltaJocabian(const float R1, const float R2, const float L1,const float L2,const float x,const float y,const float z,float *J);
void DeltaForce(const float R1, const float R2, const float L1,const float L2,const float x,const float y,const float z,float *taulist,float *forcelist);


/* void mr_Print(const char *str); */
/* void mr_PrintMatrix(float *m, const int n1, const int n2);
void mr_PrintVector(const float *v, const int n); */
void mr_AddVectors(const float *v1, const float *v2, const int n, float *v3);
void mr_MinusVectors(const float *v1, const float *v2, const int n, float *v3);
void mr_AddMatrices(const float *s1, const float *s2, const int n1, const int n2, float *d);
void mr_MinusMatrices(const float *s1, const float *s2, const int n1, const int n2, float *d);
void mr_MS(const float *m, const int n1, const int n2, const float s, float *d);
void mr_MV(const float *m, const int n1, const int n2, const float *v, float *d);
void mr_MM(const float *m1, float *m2, const int n1, const int n2, const int n3, float *d);
void mr_CopyVector(const float *s, const int n, float *d);
void mr_Transpose(const float *s, const int n1, const int n2, float *d);


#endif