#ifndef OP_ENCODER_H
#define OP_ENCODER_H

#include "NU32.h" 
#include <stdint.h>

//#define CS LATDbits.LATD10

unsigned int op_encoder_spi_master_io(unsigned int o);

void op_encoder_spi_master_init(void);

unsigned int op_encoder_hex(void);

float op_encoder_absdeg(void);

float op_encoder_deg(float z_deg);

#endif