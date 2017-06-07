#include "op_encoder.h"


unsigned int op_encoder_spi_master_io(unsigned int o) {
	SPI4BUF = o;
	while(!SPI4STATbits.SPIRBF) { // wait to receive the byte
      ;
	}
	return SPI4BUF;
}


void op_encoder_spi_master_init() {
  SPI4BUF;                  // clear the rx buffer by reading from it
  SPI4BRG = 79;             // baud rate to 80 MHz [SPI4BRG = (80M/(2*desired))-1]  desired = 0.5Mz
  SPI4STATbits.SPIROV = 0;  // clear the overflow bit
  SPI4CONbits.MODE32 = 0;   // use 32 bit mode
  SPI4CONbits.MODE16 = 1;
  SPI4CONbits.MSTEN = 1;    // master operation
  SPI4CONbits.ON = 1;       // turn on spi 4
  //TRISDbits.TRISD10 = 0;    // D10 pin as CS
  //CS = 1;
}

/* // 32 buffer
unsigned int op_encoder_hex(){
	uint32_t read_encoder = op_encoder_spi_master_io(0x12345678);
	return read_encoder;
} */

// 16 buffer
unsigned int op_encoder_hex(){
	uint16_t read1_encoder = op_encoder_spi_master_io(0x1234);
	uint16_t read2_encoder = op_encoder_spi_master_io(0x5678);
	uint16_t read3_encoder = op_encoder_spi_master_io(0x9012);
	uint32_t hex_32 = 0x0;
	hex_32 = (read2_encoder & 0x7fff) << 8;   // the 2nd bit of read2 is the 1st bit of the 23-bit data
	hex_32 = hex_32 | (read3_encoder >> 8);   // 0-7 bits are useless
	return hex_32;
}

float op_encoder_absdeg(){
	unsigned int dec = op_encoder_hex();    // 0-8388608, %8f
	float deg = (float)360*dec/8388608;
	return deg;
}

float op_encoder_deg(float z_deg){
	float abs_deg = op_encoder_absdeg();
	float deg = abs_deg - z_deg;
	if (deg<0){
		deg = 360 + deg;
	}
	return deg;
}




