#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include "NU32.h" 
#include "float_int.h"

#define CHECK LATBbits.LATB9

unsigned short spi_slave_io(unsigned short o);

unsigned int spi_slave_io_int(unsigned int o);

void spi_slave_init(void);

//four functions to send/read joint torques and angles 

float spi_read_float();

void spi_send_float(float number);


#endif//SPI_SLAVE_H