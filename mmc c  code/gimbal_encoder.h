#ifndef GIMBAL_ENCODER_H
#define GIMBAL_ENCODER_H

#include "NU32.h" 

#define CS_encoder1 LATDbits.LATD8
#define CS_encoder2 LATDbits.LATD9
#define CS_encoder3 LATDbits.LATD10

//Read three gimbal encoder data
unsigned short abs_encoder_spi_master_io(unsigned short o);

void abs_encoder_spi_master_init(void);

float abs_encoder1_absdeg(void);

float abs_encoder1_deg(float z_deg_1);

float abs_encoder2_absdeg(void);

float abs_encoder2_deg(float z_deg_2);
 
float abs_encoder3_absdeg(void);

float abs_encoder3_deg(float z_deg_3);

#endif//GIMBAL_ENCODER_H