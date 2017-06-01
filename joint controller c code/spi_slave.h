#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H
//This .h(and .c) file is used for spi communication between NU32s.(slave code) 


#include "NU32.h" 
#include "float_int.h"            //for conversion between float and int

#define CHECK LATBbits.LATB9      // spi check pin 

//use spi to send one unsigned int
unsigned int spi_slave_io_int(unsigned int o);

//initialize spi3 for communication between NU32s
void spi_slave_init(void);

//read one float number
float spi_read_float();

//send one float number
void spi_send_float(float number);


#endif//SPI_SLAVE_H