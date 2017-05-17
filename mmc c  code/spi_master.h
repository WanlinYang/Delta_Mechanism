#ifndef SPI_MASTER_H
#define SPI_MASTER_H
//This .h(and .c) file is used for spi communication between NU32s.(master code) 


#include "NU32.h" 
#include "float_int.h"//for conversion between float and int

#define CS1 LATEbits.LATE0       // chip select pin for joint controller1
#define CS2 LATEbits.LATE1       // chip select pin for joint controller2
#define CS3 LATEbits.LATE2       // chip select pin for joint controller3

#define CHECK1 PORTBbits.RB9     // spi check pin for joint controller1
#define CHECK2 PORTBbits.RB10    // spi check pin for joint controller2
#define CHECK3 PORTBbits.RB11    // spi check pin for joint controller3

//use spi to send one unsigned int
unsigned int spi_master_io_int(unsigned int o);

//initialize spi3 for communication between NU32s
void spi_master_init(void);

//read one float number
float spi_read_one_float(void);

//send one float number
void spi_send_one_float(int number);

//read three current torques from three joint controllers
void spi_read_torquelist(float *torque);

//send three desired torques to three joint controllers
void spi_send_torquelist(float *torque);

//send three nominal torques to three controllers
void spi_send_nominal_torquelist(float *torque);

//send three stiffness to three controllers
void spi_send_Stiffness(float Stiffness);

//reset spring abs encoders
void spi_reset_spring_encoders(void);

//read spring abs encoders
void spi_read_spring_encoders(float *encoderlist);

//send PWM
void spi_send_PWM(float *PWM_list);

#endif//SPI_MASTER_H