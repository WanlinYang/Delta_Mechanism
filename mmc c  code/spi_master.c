#include "NU32.h"       // constants, funcs for startup and UART
#include "spi_master.h"

//use spi to send one unsigned int
unsigned int spi_master_io_int(unsigned int o){
	SPI3BUF = o;
	while(!SPI3STATbits.SPIRBF) { // wait to receive the byte
     ;
	}
	return SPI3BUF; 
}

//initialize spi3 for communication between NU32s
void spi_master_init() {
  AD1PCFGbits.PCFG9 = 1;
  AD1PCFGbits.PCFG10 = 1;
  AD1PCFGbits.PCFG11 = 1;
  TRISBbits.TRISB9 = 1;
  TRISBbits.TRISB10 = 1;
  TRISBbits.TRISB11 = 1;
  TRISEbits.TRISE0 = 0;     // set E0 as output
  TRISEbits.TRISE1 = 0;     // set E1 as output
  TRISEbits.TRISE2 = 0;     // set E2 as output
  CS1 = 1;                  // set E0 high  
  CS2 = 1;                  // set E1 high 
  CS3 = 1;                  // set E2 high 
  SPI3BUF;                  // clear the rx buffer by reading from it
  SPI3BRG = 200;            // baud rate to ??? [SPI4BRG = (80000000/(2*desired))-1]
  SPI3STATbits.SPIROV = 0;  // clear the overflow bit
  SPI3CONbits.MODE32 = 1;   // use 32 bit mode
  SPI3CONbits.MODE16 = 0;
  SPI3CONbits.MSTEN = 1;    // master operation
  SPI3CONbits.ON = 1;       // turn on spi 3  
}

//read three current torques from three joint controller
void spi_read_torquelist(float *torquelist){
    char buffer[10];
	unsigned int temp = 0;
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000001);
	
    while(CHECK1 == 0){;}//send mode
	
	float torque_JC1 = spi_read_one_float(); //receive data
	CS1 = 1;//pull up CS pin of JC1	
	
    temp = 0;
	
    CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000001);
	
    while(CHECK2 == 0){;}//send mode
	
    float torque_JC2 = spi_read_one_float(); //receive data
	CS2 = 1;//pull up CS pin of JC2

	
	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000001);
	
    while(CHECK3 == 0){;}//send mode
	
    float torque_JC3 = spi_read_one_float(); //receive data
	CS3 = 1;//pull up CS pin of JC3
	
	*torquelist++ = torque_JC1; //store data in a list
	*torquelist++ = torque_JC2;	
	*torquelist = torque_JC3;	
}

//send three stiffness to three controllers
void spi_send_Stiffness(float Stiffness){
	unsigned int Stiffness_int = float_int_convert(Stiffness);//convert from float to int
	
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000003);
	while(CHECK1 == 0){;}//send mode
    spi_send_one_float(Stiffness_int); //send data
	while(CHECK1 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS1 = 1;//pull up CS pin of JC1
	
	CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000003);
	while(CHECK2 == 0){;}//send mode
    spi_send_one_float(Stiffness_int); //send data
	while(CHECK2 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS2 = 1;//pull up CS pin of JC2
	 
	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000003);
	while(CHECK3 == 0){;}//send mode
    spi_send_one_float(Stiffness_int); //send data
	while(CHECK3 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS3 = 1;//pull up CS pin of JC3
}

//send three desired torques to three joint controllers
void spi_send_torquelist(float *torque){
	float torque_JC1 = *torque++;
	float torque_JC2 = *torque++;	
	float torque_JC3 = *torque;		
	
	unsigned int tor_JC1_int = float_int_convert(torque_JC1);//convert from float to int
	unsigned int tor_JC2_int = float_int_convert(torque_JC2);
	unsigned int tor_JC3_int = float_int_convert(torque_JC3);
	
	
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000000);
	while(CHECK1 == 0){;}//send mode
    spi_send_one_float(tor_JC1_int); //send data
	while(CHECK1 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS1 = 1;//pull up CS pin of JC1

	
	CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000000);
	while(CHECK2 == 0){;}//send mode
    spi_send_one_float(tor_JC2_int); //send data
	while(CHECK2 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS2 = 1;//pull up CS pin of JC2
	
	
	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000000);
	while(CHECK3 == 0){;}//send mode
    spi_send_one_float(tor_JC3_int); //send data
	while(CHECK3 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS3 = 1;//pull up CS pin of JC3
	
}

//send three nominal torques to three controllers
void spi_send_nominal_torquelist(float *torque){
	float torque_JC1 = *torque++;
	float torque_JC2 = *torque++;	
	float torque_JC3 = *torque;		
	
	unsigned int tor_JC1_int = float_int_convert(torque_JC1);//convert from float to int
	unsigned int tor_JC2_int = float_int_convert(torque_JC2);
	unsigned int tor_JC3_int = float_int_convert(torque_JC3);
	
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000002);
	while(CHECK1 == 0){;}//send mode
    spi_send_one_float(tor_JC1_int); //send data
	while(CHECK1 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS1 = 1;//pull up CS pin of JC1
	
    CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000002);
	while(CHECK2 == 0){;}//send mode
    spi_send_one_float(tor_JC2_int); //send data
	while(CHECK2 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS2 = 1;//pull up CS pin of JC2
	
	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000002);
	while(CHECK3 == 0){;}//send mode
    spi_send_one_float(tor_JC3_int); //send data
	while(CHECK3 == 1){;}//send the data until check 0x00000002 sent from slave
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   //Delay for safety
	CS3 = 1;//pull up CS pin of JC3
	
}

//reset spring abs encoders
void spi_reset_spring_encoders(){
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000006);;
	while(CHECK1 == 0){;}
	while(CHECK1 == 1){;}
	CS1 = 1;//pull up CS pin of JC1
	
	CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000006);
	while(CHECK2 == 0){;}
	while(CHECK2 == 1){;}
	CS2 = 1;//pull up CS pin of JC2
	
	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000006);
	while(CHECK3 == 0){;}
	while(CHECK3 == 1){;}
	CS3 = 1;//pull up CS pin of JC3
	
}

//read spring abs encoders
void spi_read_spring_encoders(float *encoderlist){
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000004);
    while(CHECK1 == 0){;}//send mode
	float encoder_JC1_1 = spi_read_one_float(); //receive data
	CS1 = 1;//pull up CS pin of JC1	
	
	CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000004);
    while(CHECK2 == 0){;}//send mode
	float encoder_JC2_1 = spi_read_one_float(); //receive data
	CS2 = 1;//pull up CS pin of JC2

	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000004);
    while(CHECK3 == 0){;}//send mode
	float encoder_JC3_1 = spi_read_one_float(); //receive data
	CS3 = 1;//pull up CS pin of JC3
	
	CS1 = 0;//lower CS pin of JC1
	spi_master_io_int(0x01000005);
    while(CHECK1 == 0){;}//send mode
	float encoder_JC1_2 = spi_read_one_float(); //receive data
	CS1 = 1;//pull up CS pin of JC1	
	
	CS2 = 0;//lower CS pin of JC2
	spi_master_io_int(0x01000005);
    while(CHECK2 == 0){;}//send mode
	float encoder_JC2_2 = spi_read_one_float(); //receive data
	CS2 = 1;//pull up CS pin of JC2

	CS3 = 0;//lower CS pin of JC3
	spi_master_io_int(0x01000005);
    while(CHECK3 == 0){;}//send mode
	float encoder_JC3_2 = spi_read_one_float(); //receive data
	CS3 = 1;//pull up CS pin of JC3
	
	*encoderlist++ = encoder_JC1_1; //store data in a list
	*encoderlist++ = encoder_JC1_2;	
	*encoderlist++ = encoder_JC2_1;
	*encoderlist++ = encoder_JC2_2;
	*encoderlist++ = encoder_JC3_1;
	*encoderlist = encoder_JC3_2;
}

//read one float number
float spi_read_one_float(){
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 200){;}    //Delay for safety	
	unsigned int number_int = spi_master_io_int(0x01000111);//read the data (int)
	float number = int_float_convert(number_int);//convert int to float
	return number; 
}

//send one float number
void spi_send_one_float(int number){
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 10){;}    //Delay for safety
	spi_master_io_int(number);
}


