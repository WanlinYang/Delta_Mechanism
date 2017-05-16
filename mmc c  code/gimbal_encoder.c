#include "gimbal_encoder.h"                   
#include <xc.h>

unsigned short abs_encoder_spi_master_io(unsigned short o) {

	SPI4BUF = o;
	while(!SPI4STATbits.SPIRBF) { // wait to receive the byte
      ;
	}
	return SPI4BUF;
}


void abs_encoder_spi_master_init() {
  TRISDbits.TRISD8 = 0;     // set D8 as output
  TRISDbits.TRISD9 = 0;     // set D9 as output
  TRISDbits.TRISD10 = 0;    // set D10 as output
  CS_encoder1 = 1;          // set D8 high
  CS_encoder2 = 1;          // set D9 high
  CS_encoder3 = 1;          // set D10 high
  SPI4BUF;                  // clear the rx buffer by reading from it
  SPI4BRG = 50;             // baud rate to 8 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI4STATbits.SPIROV = 0;  // clear the overflow bit
  SPI4CONbits.MODE32 = 0;   // use 32 bit mode
  SPI4CONbits.MODE16 = 1;
  SPI4CONbits.MSTEN = 1;    // master operation
  SPI4CONbits.ON = 1;       // turn on spi 4  

  //TRISDbits.TRISD9 = 0;
  //PROG = 0;
  //ALIGH = 0;
}


float abs_encoder1_absdeg(){
	CS_encoder1 = 0;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	unsigned short b =  abs_encoder_spi_master_io(0x1234);    // receive count 16bit data, b is a binary number
	CS_encoder1 = 1;											 // any random number? when receiving data
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	b = b >> 4;            // last five bits are useless
	int d = b;             // d is decimal number, from 0 to 4095
	float deg = (float)360*d/4095;  // transfer to angular degree, from 0 to 360
	return deg;
}


float abs_encoder1_deg(float z_deg_1){    // input "zero" degree
	float abs_deg = abs_encoder1_absdeg();
	float deg = abs_deg - z_deg_1;
	if (deg<0){
		deg = 360 + deg;
	}
	return deg;
}

float abs_encoder2_absdeg(){
	CS_encoder2 = 0;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	unsigned short b =  abs_encoder_spi_master_io(0x1234);    // receive count 16bit data, b is a binary number
	CS_encoder2 = 1;											 // any random number? when receiving data
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	b = b >> 4;            // last five bits are useless
	int d = b;             // d is decimal number, from 0 to 4095
	float deg = (float)360*d/4095;  // transfer to angular degree, from 0 to 360
	return deg;
}


float abs_encoder2_deg(float z_deg_2){    // input "zero" degree
	float abs_deg = abs_encoder2_absdeg();
	float deg = abs_deg - z_deg_2;
	if (deg<0){
		deg = 360 + deg;
	}
	return deg;
}

float abs_encoder3_absdeg(){
	CS_encoder3 = 0;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	unsigned short b =  abs_encoder_spi_master_io(0x1234);    // receive count 16bit data, b is a binary number
	CS_encoder3 = 1;											 // any random number? when receiving data
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	b = b >> 4;            // last five bits are useless
	int d = b;             // d is decimal number, from 0 to 4095
	float deg = (float)360*d/4095;  // transfer to angular degree, from 0 to 360
	return deg;
}


float abs_encoder3_deg(float z_deg_3){    // input "zero" degree
	float abs_deg = abs_encoder3_absdeg();
	float deg = abs_deg - z_deg_3;
	if (deg<0){
		deg = 360 + deg;
	}
	return deg;
}
 