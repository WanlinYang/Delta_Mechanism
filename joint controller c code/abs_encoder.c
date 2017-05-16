#include "NU32.h"       // constants, funcs for startup and UART
#include "abs_encoder.h"


unsigned short abs_encoder_spi_master_io(unsigned short o) {

	SPI4BUF = o;
	while(!SPI4STATbits.SPIRBF) { // wait to receive the byte
      ;
	}
	return SPI4BUF;
}


void abs_encoder_spi_master_init() {
  TRISDbits.TRISD8 = 0;
  TRISDbits.TRISD9 = 0;
  TRISDbits.TRISD10 = 0;
  TRISDbits.TRISD11 = 0; 
  CS_encoder1 = 1;
  CS_encoder2 = 1; 
  PROG = 0;
  ALIGH = 0;
  SPI4BUF;                  // clear the rx buffer by reading from it
  SPI4BRG = 39;             // baud rate to 8 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI4STATbits.SPIROV = 0;  // clear the overflow bit
  SPI4CONbits.MODE32 = 0;   // use 32 bit mode
  SPI4CONbits.MODE16 = 1;
  SPI4CONbits.MSTEN = 1;    // master operation
  SPI4CONbits.ON = 1;       // turn on spi 4  

}


float abs_encoder1_absdeg(){

	CS_encoder1 = 0;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	unsigned short b = abs_encoder_spi_master_io(0x1234);
	b = abs_encoder_spi_master_io(0x1234);
	CS_encoder1 = 1;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	b = b >> 5;            // last five bits are useless
	b = b & 0b01111111111; // the 1st bit is useless
	int d = b;             // d is decimal number, from 0 to 1023
	float deg = (float)360*d/1023;  // transfer to angular degree, from 0 to 360
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

/* 	CS_encoder2 = 0;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	unsigned short b = abs_encoder_spi_master_io(0x1234);
	b = abs_encoder_spi_master_io(0x1234);
	CS_encoder2 = 1;
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() <= 21){;}
	b = b >> 5;            // last five bits are useless
	b = b & 0b01111111111; // the 1st bit is useless
	int d = b;             // d is decimal number, from 0 to 1023
	float deg = (float)360*d/1023;  // transfer to angular degree, from 0 to 360 */
	float deg = 0;
	return deg;
}

float abs_encoder2_deg(float z_deg_2){    // input "zero" degree
/* 	float abs_deg = abs_encoder2_absdeg();
	float deg = abs_deg - z_deg_2;
	if (deg<0){
		deg = 360 + deg;
	} */
	float deg = 0;
	return deg;
} 

float joint_torque(float stiffness,float z_deg_1,float z_deg_2){
	float angle_1 = abs_encoder1_deg(z_deg_1);
	float angle_2 = abs_encoder2_deg(z_deg_2);
	float tor = (angle_1 - angle_2)*stiffness;
	return tor;
}












