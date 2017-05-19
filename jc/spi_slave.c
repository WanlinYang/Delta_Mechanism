#include "NU32.h"       // constants, funcs for startup and UART
#include "spi_slave.h"


//use spi to send one unsigned int
unsigned int spi_slave_io_int(unsigned int o){
	SPI3BUF = o;
	while(!SPI3STATbits.SPIRBF) { // wait to receive the byte
     ;
	}
	return SPI3BUF;
}


//initialize spi3 for communication between NU32s
void spi_slave_init() {
  AD1PCFGbits.PCFG9 = 1;    // set B9 as a digital port
  TRISBbits.TRISB9 = 0;     // set B9 as output
  CHECK = 0;                // set B9 low 
  SPI3BUF;                  // clear the rx buffer by reading from it
  SPI3STATbits.SPIROV = 0;  // clear the overflow bit
  SPI3CONbits.MODE32 = 1;   // use 32 bit mode
  SPI3CONbits.MODE16 = 0;
  SPI3CONbits.MSTEN = 0;    // master operation
  SPI3CONbits.SSEN = 1;     // use SS3 pin  
  SPI3CONbits.ON = 1;       // turn on spi 3
}


//read one float number
float spi_read_float(){

		CHECK = 1;                                               // pull up spi check pin
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50){;}                          // delay for safety
		unsigned int number_int = spi_slave_io_int(0x00000022);  // read data (int form)
	    CHECK = 0;                                               // lower spi check pin
		_CP0_SET_COUNT(0);
	    while(_CP0_GET_COUNT() < 300){;}                         // delay for safety
		float number = int_float_convert(number_int);            // convert int to float  
		return number;
}


//send one float number
void spi_send_float(float number){

		unsigned int number_int = float_int_convert(number);     // covert float to int
		CHECK = 1;                                               // pull up spi check pin
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50){;}                          // delay for safety
		spi_slave_io_int(number_int);                            // send data (int form)
		CHECK = 0;                                               // lower spi check pin
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 300){;}                         // delay for safety
	
}

















