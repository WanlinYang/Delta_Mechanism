#ifndef SENDCONTROL_H
#define SENDCONTROL_H

#include "NU32.h" 

//PWM initialize
void send_control_ini(void);

//change PWM
void send_control_signal(int value);

#endif//SENDCONTROL_H