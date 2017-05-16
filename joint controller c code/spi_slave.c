#include "NU32.h"       // constants, funcs for startup and UART
#include "spi_slave.h"


unsigned short spi_slave_io(unsigned short o) {

	SPI3BUF = o;
	while(!SPI3STATbits.SPIRBF) { // wait to receive the byte
		;
	}
	return SPI3BUF;
}


unsigned int spi_slave_io_int(unsigned int o){
	SPI3BUF = o;
	while(!SPI3STATbits.SPIRBF) { // wait to receive the byte
     ;
	}
	return SPI3BUF;
}


void spi_slave_init() {
  AD1PCFGbits.PCFG9 = 1;
  TRISBbits.TRISB9 = 0;
  CHECK = 0;
  SPI3BUF;                  // clear the rx buffer by reading from it
  SPI3STATbits.SPIROV = 0;  // clear the overflow bit
  SPI3CONbits.MODE32 = 1;   // use 32 bit mode
  SPI3CONbits.MODE16 = 0;
  SPI3CONbits.MSTEN = 0;    // master operation
  SPI3CONbits.ON = 1;       // turn on spi 3
  SPI3CONbits.SSEN = 1;     // use SS3 pin  
  TRISEbits.TRISE2 = 1;
}



float spi_read_float(){

		CHECK = 1;
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50){;}//Delay for safety
		unsigned int number_int = spi_slave_io_int(0x00000022);
	    CHECK = 0;
		_CP0_SET_COUNT(0);
	    while(_CP0_GET_COUNT() < 300){;}
		float number = int_float_convert(number_int);
		return number;
}

void spi_send_float(float number){

		unsigned int number_int = float_int_convert(number);
		CHECK = 1;
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50){;}
		spi_slave_io_int(number_int);
		CHECK = 0;
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 300){;}//Delay for safety
	
}

















