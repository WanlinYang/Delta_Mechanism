#include "NU32.h"       // constants, funcs for startup and UART
#include "abs_encoder.h"

//use spi to send one unsigned short
unsigned short abs_encoder_spi_master_io(unsigned short o) {

	SPI4BUF = o;
	while(!SPI4STATbits.SPIRBF) { // wait to receive the byte
      ;
	}
	return SPI4BUF;
}


//initialize spi4 for communication between NU32 and three gimbal encoders
/* void abs_encoder_spi_master_init() {
  TRISDbits.TRISD8 = 0;     // set D8 as output
  TRISDbits.TRISD9 = 0;     // set D9 as output
  TRISDbits.TRISD10 = 0;    // set D10 as output
  TRISDbits.TRISD11 = 0;    // set D11 as output
  CS_encoder1 = 1;          // set D10 high
  CS_encoder2 = 1;          // set D11 high
  SPI4BUF;                  // clear the rx buffer by reading from it
  SPI4BRG = 39;             // baud rate to 100 kHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI4STATbits.SPIROV = 0;  // clear the overflow bit
  SPI4CONbits.MODE32 = 0;   // use 16 bit mode
  SPI4CONbits.MODE16 = 1;
  SPI4CONbits.MSTEN = 1;    // master operation
  SPI4CONbits.ON = 1;       // turn on spi 4  

} */

//read absolute data from spring encoder 1
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

//read relative data from spring encoder 1 by using z_deg_1 as a reference
float abs_encoder1_deg(float z_deg_1){    // input "zero" degree
	float abs_deg = abs_encoder1_absdeg();
	float deg = abs_deg - z_deg_1;
	if (deg<0){
		deg = 360 + deg;
	}
	return deg;
}

//read absolute data from spring encoder 2
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

//read relative data from spring encoder 2 by using z_deg_2 as a reference
float abs_encoder2_deg(float z_deg_2){    // input "zero" degree
/* 	float abs_deg = abs_encoder2_absdeg();
	float deg = abs_deg - z_deg_2;
	if (deg<0){
		deg = 360 + deg;
	} */
	float deg = 0;
	return deg;
} 

//calcalate joint torque by two spring encoders
float joint_torque(float stiffness,float z_deg_1,float z_deg_2){
	float angle_1 = abs_encoder1_deg(z_deg_1);   // read spring encoder 1
	float angle_2 = abs_encoder2_deg(z_deg_2);   // read spring encoder 2
	float tor = (angle_1 - angle_2)*stiffness;   // calculate torque
	return tor;
}












