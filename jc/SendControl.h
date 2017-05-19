#ifndef SENDCONTROL_H
#define SENDCONTROL_H

#include "NU32.h" 

//PWM signal initialization
void send_control_ini(void);

//change duty cycle of PWM
void send_control_signal(int value);

#endif//SENDCONTROL_H