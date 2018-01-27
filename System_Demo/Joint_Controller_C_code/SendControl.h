#ifndef SENDCONTROL_H
#define SENDCONTROL_H

#include "NU32.h" 

#define SAMPLE_TIME 10

//PWM signal initialization
void send_control_ini(void);

//change duty cycle of PWM
void send_control_signal(int value);

//read ADC output(we will use pin B12)
unsigned int adc_sample_convert(int pin);

#endif//SENDCONTROL_H