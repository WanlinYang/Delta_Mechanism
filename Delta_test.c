#include "Delta.h"


void test_DeltaIkin() {
  char str[64];
  double R1 = 1;
  double R2 = 0.5;
  double L1 = 1;
  double L2 = 1;
  double x = 0.3;
  double y = 0.3;
  double z = 1.5;
  double thetalist[3];
  
  DeltaIkin(R1,R2,L1,L2,x,y,z,thetalist);
  sprintf(str,"Inverse kinematics test:\n");
  mr_Print(str);
  sprintf(str,"R1 = %.1f\n",R1);
  mr_Print(str);
  sprintf(str,"R2 = %.1f\n",R2);
  mr_Print(str);
  sprintf(str,"L1 = %.1f\n",L1);
  mr_Print(str);
  sprintf(str,"L2 = %.1f\n",L2);
  mr_Print(str); 
  sprintf(str,"x = %.1f\n",x);
  mr_Print(str); 
  sprintf(str,"y = %.1f\n",y);
  mr_Print(str); 
  sprintf(str,"z = %.1f\n",z);
  mr_Print(str); 
  sprintf(str,"thetalist = \n");
  mr_Print(str);  
  mr_PrintVector(thetalist,3);
}

void test_DeltaFkin() {
  char str[64];
  double R1 = 1;
  double R2 = 0.5;
  double L1 = 1;
  double L2 = 1;
  double thetalist[3] = {0.5333,0.3388,-0.0516};
  double p0[3] = {0.2,0.4,1.3};
  double p[3];
  
  DeltaFkin(R1,R2,L1,L2,thetalist,p0,p);
  sprintf(str,"Forward kinematics test:\n");
  mr_Print(str);
  sprintf(str,"R1 = %.1f\n",R1);
  mr_Print(str);
  sprintf(str,"R2 = %.1f\n",R2);
  mr_Print(str);
  sprintf(str,"L1 = %.1f\n",L1);
  mr_Print(str);
  sprintf(str,"L2 = %.1f\n",L2);
  mr_Print(str); 
  sprintf(str,"thetalist = \n");
  mr_Print(str);  
  mr_PrintVector(thetalist,3);
  sprintf(str,"p_guess = \n");
  mr_Print(str);  
  mr_PrintVector(p0,3);  
  sprintf(str,"p = \n");
  mr_Print(str);  
  mr_PrintVector(p,3);   
}

void test_DeltaJocabian() {
  char str[64];
  double R1 = 1;
  double R2 = 0.5;
  double L1 = 1;
  double L2 = 1;
  double x = 0.3;
  double y = 0.3;
  double z = 1.5;
  double J[9];
  DeltaJocabian(R1,R2,L1,L2,x,y,z,J);
  sprintf(str,"Jacobian test:\n");
  mr_Print(str);
  sprintf(str,"R1 = %.1f\n",R1);
  mr_Print(str);
  sprintf(str,"R2 = %.1f\n",R2);
  mr_Print(str);
  sprintf(str,"L1 = %.1f\n",L1);
  mr_Print(str);
  sprintf(str,"L2 = %.1f\n",L2);
  mr_Print(str); 
  sprintf(str,"x = %.1f\n",x);
  mr_Print(str); 
  sprintf(str,"y = %.1f\n",y);
  mr_Print(str); 
  sprintf(str,"z = %.1f\n",z);
  mr_Print(str); 
  sprintf(str,"J = \n");
  mr_Print(str);  
  mr_PrintMatrix(J,3,3);
}

int main(void) {
  
  
  test_DeltaIkin(); 
  test_DeltaFkin();
  test_DeltaJocabian();
  return(0);
  
  
}