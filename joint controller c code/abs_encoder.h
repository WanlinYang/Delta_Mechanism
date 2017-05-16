#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include "NU32.h" 

#define PROG LATDbits.LATD8      
#define ALIGH LATDbits.LATD9     
#define CS_encoder1 LATDbits.LATD10       // chip select pin
#define CS_encoder2 LATDbits.LATD11       // chip select pin

unsigned short abs_encoder_spi_master_io(unsigned short o);

void abs_encoder_spi_master_init(void);

float abs_encoder1_absdeg(void);

float abs_encoder1_deg(float z_deg_1);//spring side angle

float abs_encoder2_absdeg(void);

float abs_encoder2_deg(float z_deg_2);//spring side angle

float joint_torque(float stiffness,float z_deg_1,float z_deg_2);//torque


#endif//SPI_MASTER_H