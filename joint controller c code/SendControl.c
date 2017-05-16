#include "SendControl.h"


void send_control_ini(void){
  //Timer3+OC1(PWM)
  T3CONbits.TCKPS = 0;     
  PR3 = 3999;              
  TMR3 = 0;                
  OC1CONbits.OCM = 0b110;  
  OC1CONbits.OCTSEL = 1;             
  OC1RS = 0;             
  OC1R = 0;              
  T3CONbits.ON = 1;   
  OC1CONbits.ON = 1;
  
  
  //digital output(direction bit)
  TRISDbits.TRISD6 = 0; 
	
	
}


void send_control_signal(int value){
	 OC1RS =  value;
}