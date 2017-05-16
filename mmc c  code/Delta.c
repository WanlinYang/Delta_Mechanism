#include "Delta.h"

/* void mr_PrintMatrix(double *m, const int n1, const int n2) {
  int i,j;
  char str[16];

  sprintf(str,"\n");
  mr_Print(str);
  for (i=0; i<n1; i++) {
    for (j=0; j<n2; j++) {
      sprintf(str,"%8.4f ",*m++);
      mr_Print(str);
    }
    sprintf(str,"\n");
    mr_Print(str);
  }
} */

/* void mr_Print(const char *str) {
  fprintf(stdout,"%s",str);
} */

/* void mr_PrintVector(const double *v, const int n) {
  int i;
  char str[16];

  sprintf(str,"\n");
  mr_Print(str);
  for (i=0; i<n; i++) {
    sprintf(str,"%10.4f ",*v++);
    mr_Print(str);
  }
  sprintf(str,"\n");
  mr_Print(str);
} */

void mr_AddVectors(const double *v1, const double *v2, const int n, double *v3) {
  int i;

  for (i=0; i<n; i++)
    *v3++ = *v1++ + *v2++;
}

void mr_CopyVector(const double *s, const int n, double *d) {
  int i;

  for (i=0; i<n; i++) {
    *d++ = *s++;
  }
}

void mr_MinusVectors(const float *v1, const float *v2, const int n, float *v3){
  int i;

  for (i=0; i<n; i++)
    *v3++ = *v1++ - *v2++;
}

void mr_AddMatrices(const double *s1, const double *s2, const int n1, const int n2, double *d) {
  int i,len;

  len = n1*n2;
  for (i=0; i<len; i++)
    *d++ = *s1++ + *s2++;
}

void mr_Transpose(const double *s, const int n1, const int n2, double *d) {
  int i,j,nextcol;

  nextcol = 1-n1*n2;
  for (i=0; i<n1; i++) {
    for (j=0; j<n2; j++) {
      *d = *s++;
      d += n1;
    }
    d += nextcol;
  }
}

void mr_MinusMatrices(const double *s1, const double *s2, const int n1, const int n2, double *d) {
  int i,len;

  len = n1*n2;
  for (i=0; i<len; i++)
    *d++ = *s1++ - *s2++;
}

void mr_MS(const double *m, const int n1, const int n2, const double s, double *d) {
  int i,len;

  len = n1*n2;
  for (i=0; i<len; i++)
    *d++ = (*m++)*s;
}

void mr_MV(const double *m, const int n1, const int n2, const double *v,
	   double *d) {
  int i,j;
  double sum;

  for (i=0; i<n1; i++) {
    sum = 0;
    for (j=0; j<n2; j++) 
      sum += (*m++)*(*v++);
    *d++ = sum;
    v -= n2;
  }
}

void mr_MM(const double *m1, double *m2, const int n1, const int n2, const int n3, double *d) {
  int i,j,k,nextm2col=1-n2*n3;
  double sum,*m2t;

  m2t = m2;
  for (i=0; i<n1; i++) {
    for (j=0; j<n3; j++) {
      sum = 0;
      for (k=0; k<n2; k++) {
        sum += (*m1++)*(*m2t);
	m2t += n3;
      }
      *d++ = sum;
      m2t += nextm2col;
      m1 -= n2;
    }
    m2t = m2;
    m1 += n2;
  }
}

void Inverse3x3(const double *s, double *d) {
	
	double a1 = *s++; double a2 = *s++; double a3 = *s++;
	double b1 = *s++; double b2 = *s++; double b3 = *s++;
	double c1 = *s++; double c2 = *s++; double c3 = *s;
	
	double d11, d12, d13, d21, d22, d23, d31, d32, d33;
	
	double det = -a3*b2*c1 + a2*b3*c1 + a3*b1*c2 - a1*b3*c2 - a2*b1*c3 + a1*b2*c3;
	
	d11 = -(b3*c2 - b2*c3)/det;
	d12 = (a3*c2 - a2*c3)/det;
	d13 = -(a3*b2 - a2*b3)/det;
	d21 = (b3*c1 - b1*c3)/det;
	d22 = -(a3*c1 - a1*c3)/det;
	d23 = (a3*b1 - a1*b3)/det;
	d31 = -(b2*c1 - b1*c2)/det;
	d32 = (a2*c1 - a1*c2)/det;
	d33 = -(a2*b1 - a1*b2)/det;
	
	*d++ = d11; *d++ = d12; *d++ = d13;
	*d++ = d21; *d++ = d22; *d++ = d23;
	*d++ = d31; *d++ = d32; *d = d33;


}

void DeltaoFunction(double *x, double *blist, double *plist, double *Llist, double L2, double *d){
	
	double x1 = *x++; double x2 = *x++; double x3 = *x;
	
	double b11 = *blist++; double b12 = *blist++; double b13 = *blist++;
	double b21 = *blist++; double b22 = *blist++; double b23 = *blist++;
	double b31 = *blist++; double b32 = *blist++; double b33 = *blist;
	
	double p11 = *plist++; double p12 = *plist++; double p13 = *plist++;
	double p21 = *plist++; double p22 = *plist++; double p23 = *plist++;
	double p31 = *plist++; double p32 = *plist++; double p33 = *plist;
	
	double L11 = *Llist++; double L12 = *Llist++; double L13 = *Llist++;
	double L21 = *Llist++; double L22 = *Llist++; double L23 = *Llist++;
	double L31 = *Llist++; double L32 = *Llist++; double L33 = *Llist;

	*d++ = (x1+p11-L11-b11)*(x1+p11-L11-b11)+(x2+p12-L12-b12)*(x2+p12-L12-b12)+(x3+p13-L13-b13)*(x3+p13-L13-b13)-L2*L2;
	*d++ = (x1+p21-L21-b21)*(x1+p21-L21-b21)+(x2+p22-L22-b22)*(x2+p22-L22-b22)+(x3+p23-L23-b23)*(x3+p23-L23-b23)-L2*L2;
	*d = (x1+p31-L31-b31)*(x1+p31-L31-b31)+(x2+p32-L32-b32)*(x2+p32-L32-b32)+(x3+p33-L33-b33)*(x3+p33-L33-b33)-L2*L2;

}

void DeltadFunction(double *x, double *blist, double *plist, double *Llist, double L2, double *df){
	double h = 0.0001;
	double df11, df12=0, df13=0, df21=0, df22=0, df23=0, df31=0, df32=0, df33=0;
	
	double b11 = *blist++; double b12 = *blist++; double b13 = *blist++;
	double b21 = *blist++; double b22 = *blist++; double b23 = *blist++;
	double b31 = *blist++; double b32 = *blist++; double b33 = *blist;
	double blistf[9] = {b11, b12, b13, b21, b22, b23, b31, b32, b33};
	
	double p11 = *plist++; double p12 = *plist++; double p13 = *plist++;
	double p21 = *plist++; double p22 = *plist++; double p23 = *plist++;
	double p31 = *plist++; double p32 = *plist++; double p33 = *plist;
	double plistf[9] = {p11, p12, p13, p21, p22, p23, p31, p32, p33};
	
	double L11 = *Llist++; double L12 = *Llist++; double L13 = *Llist++;
	double L21 = *Llist++; double L22 = *Llist++; double L23 = *Llist++;
	double L31 = *Llist++; double L32 = *Llist++; double L33 = *Llist;
	double Llistf[9] = {L11, L12, L13, L21, L22, L23, L31, L32, L33};
	
	double x1 = *x++; double x2 = *x++; double x3 = *x;
	double xf[3] = {x1, x2, x3};
	double xtemp_plus[3]; double xtemp_minus[3];
	double ftemp_plus[3]; double ftemp_minus[3];
	double ftemp_d[3];
	double dftemp[3];
	
	double htemp[3] = {h, 0, 0};
	mr_AddVectors(xf, htemp, 3, xtemp_plus);
	mr_MinusVectors(xf, htemp, 3, xtemp_minus);	
	DeltaoFunction(xtemp_plus, blistf, plistf, Llistf, L2, ftemp_plus);
	DeltaoFunction(xtemp_minus, blistf, plistf, Llistf, L2, ftemp_minus);
	mr_MinusVectors(ftemp_plus, ftemp_minus, 3, ftemp_d);
	mr_MS(ftemp_d, 3, 3, 1/(2*h), dftemp);
	df11 = dftemp[0]; df21 = dftemp[1]; df31 = dftemp[2];

	htemp[0] = 0;  htemp[1] = h; htemp[2] = 0;
	mr_AddVectors(xf, htemp, 3, xtemp_plus);
	mr_MinusVectors(xf, htemp, 3, xtemp_minus);	
	DeltaoFunction(xtemp_plus, blistf, plistf, Llistf, L2, ftemp_plus);
	DeltaoFunction(xtemp_minus, blistf, plistf, Llistf, L2, ftemp_minus);
	mr_MinusVectors(ftemp_plus, ftemp_minus, 3, ftemp_d);
	mr_MS(ftemp_d, 3, 3, 1/(2*h), dftemp);
	df12 = dftemp[0]; df22 = dftemp[1]; df32 = dftemp[2];
	
	htemp[0] = 0; htemp[1] = 0; htemp[2] = h;
	mr_AddVectors(xf, htemp, 3, xtemp_plus);
	mr_MinusVectors(xf, htemp, 3, xtemp_minus);	
	DeltaoFunction(xtemp_plus, blistf, plistf, Llistf, L2, ftemp_plus);
	DeltaoFunction(xtemp_minus, blistf, plistf, Llistf, L2, ftemp_minus);
	mr_MinusVectors(ftemp_plus, ftemp_minus, 3, ftemp_d);
	mr_MS(ftemp_d, 3, 3, 1/(2*h), dftemp);
	df13 = dftemp[0]; df23 = dftemp[1]; df33 = dftemp[2];	
	
	*df++ = df11; *df++ = df12;  *df++ = df13;
	*df++ = df21; *df++ = df22;  *df++ = df23;
	*df++ = df31; *df++ = df32;  *df = df33;
}

void NewtonMethod(double *x, double *blist, double *plist, double *Llist, double L2, double *d){
	
	double b11 = *blist++; double b12 = *blist++; double b13 = *blist++;
	double b21 = *blist++; double b22 = *blist++; double b23 = *blist++;
	double b31 = *blist++; double b32 = *blist++; double b33 = *blist;
	double blistf[9] = {b11, b12, b13, b21, b22, b23, b31, b32, b33};
	
	double p11 = *plist++; double p12 = *plist++; double p13 = *plist++;
	double p21 = *plist++; double p22 = *plist++; double p23 = *plist++;
	double p31 = *plist++; double p32 = *plist++; double p33 = *plist;
	double plistf[9] = {p11, p12, p13, p21, p22, p23, p31, p32, p33};
	
	double L11 = *Llist++; double L12 = *Llist++; double L13 = *Llist++;
	double L21 = *Llist++; double L22 = *Llist++; double L23 = *Llist++;
	double L31 = *Llist++; double L32 = *Llist++; double L33 = *Llist;
	double Llistf[9] = {L11, L12, L13, L21, L22, L23, L31, L32, L33};
	
	double x1 = *x++; double x2 = *x++; double x3 = *x;
	double xf[3] = {x1, x2, x3};
	
	double y[3]; double dy[9]; double invdy[9]; double s[3]; double xp1[3];
	double epsilon = 0.0001;
	
	DeltaoFunction(xf, blistf, plistf, Llistf, L2, y);
	DeltadFunction(xf, blistf, plistf, Llistf, L2, dy);
	
	while (1){
		Inverse3x3(dy, invdy);
		mr_MV(invdy, 3, 3, y, s);
		mr_MinusVectors(xf, s, 3, xp1);
		mr_CopyVector(xp1, 3, xf);
		DeltaoFunction(xf, blistf, plistf, Llistf, L2, y);
		double y1temp = y[0];  double y2temp = y[1];  double y3temp = y[2];
		if (y1temp<epsilon && y2temp<epsilon && y3temp<epsilon )
			break;
		DeltadFunction(xf, blistf, plistf, Llistf, L2, dy);
	}
	mr_CopyVector(xf, 3, d);
}

void DeltaFkin(const double R1, const double R2, const double L1, const double L2, double *thetalist, double *p0, double *p){
	double th1 = *thetalist++, th2 = *thetalist++, th3 = *thetalist;
	double blist[9] = {R1, 0, 0, -R1*0.5, R1*0.866, 0, -R1*0.5, -R1*0.866, 0};
	double plist[9] = {R2, 0, 0, -R2*0.5, R2*0.866, 0, -R2*0.5, -R2*0.866, 0};
	double Llist[9] = {L1*sin(th1), 0, L1*cos(th1),
					-L1*0.5*sin(th2),L1*0.866*sin(th2),L1*cos(th2),
					-L1*0.5*sin(th3),-L1*0.866*sin(th3),L1*cos(th3)};
					
	double x1 = *p0++, x2 = *p0++, x3 = *p0++;
	double xlist[3] = {x1, x2, x3};
	NewtonMethod(xlist, blist, plist, Llist, L2, p);
	
}

void DeltaIkin(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *thetalist){
	double bmplist[9],G[3],E,F[3],tp,tm,thetap,thetam ;
	int i,j;
	bmplist[0]=R1-R2; bmplist[1]=0; bmplist[2]=0;
	bmplist[3]= -(R1-R2)*0.5; bmplist[4]=(R1-R2)*0.866; bmplist[5]=0;
	bmplist[6]= -(R1-R2)*0.5; bmplist[7]=-(R1-R2)*0.866; bmplist[8]=0;
	for(i=0; i<3; i++){
		G[i] = L2*L2-L1*L1-(x-bmplist[3*i])*(x-bmplist[3*i])-(y-bmplist[3*i+1])*(y-bmplist[3*i+1])-z*z;
	}
	
	E = 2*z*L1;
	F[0]= 2*L1*(x-bmplist[0]); 
	F[1]= 2*L1*(-(x-bmplist[3])*0.5+(y-bmplist[4])*0.866);
	F[2]= 2*L1*(-(x-bmplist[6])*0.5-(y-bmplist[7])*0.866);
    for(j=0; j<3; j++){
		tp = (-F[j]+sqrt(E*E+F[j]*F[j]-G[j]*G[j]))/(G[j]-E);
		tm = (-F[j]-sqrt(E*E+F[j]*F[j]-G[j]*G[j]))/(G[j]-E);
		thetap = 2*atan(tp);
		thetam = 2*atan(tm);
		if(-PI/4 <= thetap && thetap <= PI/2){
			*thetalist = thetap;
		}
		if(-PI/4 <= thetam && thetam <= PI/2){
			*thetalist = thetam;
		}
		*thetalist++;
	}
}

void DeltaJocabian(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *J){
	double pmbmllist[9],A[9],B[9],thetalist[3],InvA[9];
	DeltaIkin(R1,R2,L1,L2,x,y,z,thetalist);
    pmbmllist[0] = R2-R1-L1*sin(thetalist[0]); 
    pmbmllist[1] = 0; 
    pmbmllist[2] = -L1*cos(thetalist[0]);	
    pmbmllist[3] = (R1-R2+L1*sin(thetalist[1]))*0.5; 	
    pmbmllist[4] = -(R1-R2+L1*sin(thetalist[1]))*0.866; 
    pmbmllist[5] = -L1*cos(thetalist[1]); 
    pmbmllist[6] = (R1-R2+L1*sin(thetalist[2]))*0.5; 
    pmbmllist[7] = (R1-R2+L1*sin(thetalist[2]))*0.866;	
    pmbmllist[8] = -L1*cos(thetalist[2]); 
	
	A[0] = x + pmbmllist[0]; A[1] = y + pmbmllist[1]; A[2] = z + pmbmllist[2];
	A[3] = x + pmbmllist[3]; A[4] = y + pmbmllist[4]; A[5] = z + pmbmllist[5];
	A[6] = x + pmbmllist[6]; A[7] = y + pmbmllist[7]; A[8] = z + pmbmllist[8];
	
	B[0] = -z*L1*sin(thetalist[0])+(x+R2-R1)*L1*cos(thetalist[0]); 
	B[1] = 0; B[2] = 0; B[3] = 0; 
	B[4] = -z*L1*sin(thetalist[1])+(-0.5*x+0.25*(R2-R1)+0.866*y+0.75*(R2-R1))*cos(thetalist[1]);
	B[5] = 0; B[6] = 0; B[7] = 0; 
	B[8] = -z*L1*sin(thetalist[2])+(-0.5*x+0.25*(R2-R1)-0.866*y+0.75*(R2-R1))*cos(thetalist[2]);
	
	Inverse3x3(A,InvA);
	mr_MM(InvA,B,3,3,3,J);
}

void DeltaForce(const double R1, const double R2, const double L1,const double L2,const double x,const double y,const double z,double *taulist,double *forcelist){
	double J[9],Jtemp[9],Jtemp2[9];
	DeltaJocabian(R1,R2,L1,L2,x,y,z,J);
	Inverse3x3(J,Jtemp);
	mr_Transpose(Jtemp,3,3,Jtemp2);
	mr_MV(Jtemp2,3,3,taulist,forcelist);
}

