#ifndef GIMBAL_ENCODER_H
#define GIMBAL_ENCODER_H

#include "NU32.h" 

#define CS_encoder1 LATDbits.LATD8
#define CS_encoder2 LATDbits.LATD9
#define CS_encoder3 LATDbits.LATD10

//use spi to send one unsigned short
unsigned short abs_encoder_spi_master_io(unsigned short o);

//initialize spi4 for communication between NU32 and three gimbal encoders
void abs_encoder_spi_master_init(void);

//read absolute data from gimbal encoder 1
float abs_encoder1_absdeg(void);

//read relative data from gimbal encoder 1 by using z_deg_1 as a reference
float abs_encoder1_deg(float z_deg_1);

//read absolute data from gimbal encoder 2
float abs_encoder2_absdeg(void);

//read relative data from gimbal encoder 2 by using z_deg_2 as a reference
float abs_encoder2_deg(float z_deg_2);
 
//read absolute data from gimbal encoder 3
float abs_encoder3_absdeg(void);

//read relative data from gimbal encoder 3 by using z_deg_3 as a reference
float abs_encoder3_deg(float z_deg_3);

#endif//GIMBAL_ENCODER_H