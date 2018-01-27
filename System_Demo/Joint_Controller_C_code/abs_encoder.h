#ifndef ABS_ENCODER_H
#define ABS_ENCODER_H

#include "NU32.h" 
   
#define CS_encoder1 LATDbits.LATD10       // chip select pin for abs encoder 1
#define CS_encoder2 LATDbits.LATD11       // chip select pin for abs encoder 2

//use spi to send one unsigned short
unsigned short abs_encoder_spi_master_io(unsigned short o);

//initialize spi4 for communication between NU32 and three gimbal encoders
/* void abs_encoder_spi_master_init(void); */

//read absolute data from spring encoder 1
float abs_encoder1_absdeg(void);

//read relative data from spring encoder 1 by using z_deg_1 as a reference
float abs_encoder1_deg(float z_deg_1);//spring side angle

//read absolute data from spring encoder 2
float abs_encoder2_absdeg(void);

//read relative data from spring encoder 2 by using z_deg_2 as a reference
float abs_encoder2_deg(float z_deg_2);//spring side angle

//calcalate joint torque by two spring encoders
float joint_torque(float stiffness,float z_deg_1,float z_deg_2);


#endif//ABS_ENCODER_H