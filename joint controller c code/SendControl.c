#include "SendControl.h"


//PWM signal initialization
void send_control_ini(void){
	//Timer2
	PR2 = 249;                  // enable counting to max value of 249            
	TMR2 = 0;                   // set the timer count to zero
	T2CONbits.TCKPS = 6;        // set prescale ratio to be 1:64
	T2CONbits.ON = 1;           // turn on Timer2
	IPC2bits.T2IP = 4;          // set INT2 to priority 4
	IPC2bits.T2IS = 0;          // set INT2 to subpriority 0
	IFS0bits.T2IF = 0;          // clear interrupt flag
	IEC0bits.T2IE = 1;          // enable interrupt
	  
	//Timer3+OC1(PWM)
	T3CONbits.TCKPS = 0;        // set prescale ratio to be 1:1
	PR3 = 3999;                 // enable counting to max value of 3999
	TMR3 = 0;                   // set the timer count to zero
	OC1CONbits.OCM = 0b110;     // PWM mode with fault pin disabled
	OC1CONbits.OCTSEL = 1;      // use Timer3 for OC1
	OC1RS = 0;                  // duty cycle = 0
	OC1R = 0;                   // duty cycle = 0
	T3CONbits.ON = 1;           // turn on Timer3
	OC1CONbits.ON = 1;          // turn on OC1
  
	//digital output(direction bit)
	TRISDbits.TRISD6 = 0;       // set D6 as a output port
  
	//Timer4
	PR4 = 6249;
	TMR4 = 0;
	T4CONbits.TCKPS = 6;
    IPC4bits.T4IP = 5;
    IPC4bits.T4IS = 0;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
	T4CONbits.ON = 1; 
}

//read ADC output(we will use pin B12)
unsigned int adc_sample_convert(int pin){
	unsigned int elapsed = 0, finish_time = 0;
	AD1CHSbits.CH0SA = pin;
	AD1CON1bits.SAMP = 1;
	elapsed = _CP0_GET_COUNT();
	finish_time = elapsed + SAMPLE_TIME;
	while(_CP0_GET_COUNT()<finish_time){;}  // sample for more than 250 ns
	AD1CON1bits.SAMP = 0;
	while(!AD1CON1bits.DONE){;}
	return ADC1BUF0;
}


//change duty cycle of PWM
void send_control_signal(int PWM_counts){
	 
	    if (PWM_counts >= 0)
		{
			OC1RS = PWM_counts*40;   // duty cycle = PWM_counts*40
			LATDbits.LATD6 = 0;      // set direction bit
		}
		else
		{
			OC1RS = -PWM_counts*40;  // duty cycle = -PWM_counts*40
			LATDbits.LATD6 = 1;      // set direction bit
		}
}