#include "SendControl.h"


void send_control_ini(void){
  //Timer2
  PR2 = 249;
  TMR2 = 0;
  T2CONbits.TCKPS = 6;
  IPC2bits.T2IP = 4;
  IPC2bits.T2IS = 0;
  IFS0bits.T2IF = 0;
  IEC0bits.T2IE = 1;
  
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
  T2CONbits.ON = 1;
  //digital output(direction bit)
  TRISDbits.TRISD6 = 0; 
}


void send_control_signal(int PWM_counts){
	 
	    if (PWM_counts >= 0)
		{
			OC1RS = PWM_counts*40; 
			LATDbits.LATD6 = 1;
		}
		else
		{
			OC1RS = -PWM_counts*40; 
			LATDbits.LATD6 = 0;
		}
}