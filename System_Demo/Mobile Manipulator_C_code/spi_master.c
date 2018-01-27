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
  AD1PCFGbits.PCFG9 = 1;    // set B9 as a digital port
  AD1PCFGbits.PCFG10 = 1;   // set B10 as a digital port
  AD1PCFGbits.PCFG11 = 1;   // set B11 as a digital port
  TRISBbits.TRISB9 = 1;     // set B9 as input
  TRISBbits.TRISB10 = 1;    // set B10 as input
  TRISBbits.TRISB11 = 1;    // set B11 as input
  TRISEbits.TRISE0 = 0;     // set E0 as output
  TRISEbits.TRISE1 = 0;     // set E1 as output
  TRISEbits.TRISE2 = 0;     // set E2 as output
  CS1 = 1;                  // set E0 high  
  CS2 = 1;                  // set E1 high 
  CS3 = 1;                  // set E2 high 
  SPI3BUF;                  // clear the rx buffer by reading from it
  SPI3BRG = 0x4;            // baud rate to 200kHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI3STATbits.SPIROV = 0;  // clear the overflow bit
  SPI3CONbits.MODE32 = 1;   // use 32 bit mode
  SPI3CONbits.MODE16 = 0;
  SPI3CONbits.MSTEN = 1;    // master operation
  SPI3CONbits.ON = 1;       // turn on spi 3  
}

//read three current torques from three joint controller
void spi_read_torquelist(float *torquelist){
    char buffer[10];
	CS1 = 0;                                 // lower CS pin of JC1
	spi_master_io_int(0x01000001);           // send operating mode to JC1
    while(CHECK1 == 0){;}                    // check if JC1 is ready to send data
	float torque_JC1 = spi_read_one_float(); // receive data
	CS1 = 1;                                 // pull up CS pin of JC1	
	
    CS2 = 0;                                 // lower CS pin of JC2
	spi_master_io_int(0x01000001);           // send operating mode to JC2
    while(CHECK2 == 0){;}                    // check if JC2 is ready to send data
    float torque_JC2 = spi_read_one_float(); // receive data
	CS2 = 1;                                 // pull up CS pin of JC2

	CS3 = 0;                                 // lower CS pin of JC3
	spi_master_io_int(0x01000001);           // send operating mode to JC3
    while(CHECK3 == 0){;}                    // check if JC3 is ready to send data
    float torque_JC3 = spi_read_one_float(); // receive data
	CS3 = 1;                                 // pull up CS pin of JC3
	
	*torquelist++ = torque_JC1;              //store data in a list
	*torquelist++ = torque_JC2;	
	*torquelist = torque_JC3;	
}

//send three stiffness to three controllers
void spi_send_Stiffness(float Stiffness){
	unsigned int Stiffness_int = float_int_convert(Stiffness);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000003);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(Stiffness_int); // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000003);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(Stiffness_int); // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000003);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(Stiffness_int); // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}

//send three desired torques to three joint controllers
void spi_send_torquelist(float *torque){
	float torque_JC1 = *torque++;
	float torque_JC2 = *torque++;	
	float torque_JC3 = *torque;		
	
	unsigned int tor_JC1_int = float_int_convert(torque_JC1);// convert from float to int for spi communication
	unsigned int tor_JC2_int = float_int_convert(torque_JC2);
	unsigned int tor_JC3_int = float_int_convert(torque_JC3);
	
	
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000000);   // send operating mode to JC1
	while(CHECK1 == 0){;}            // check if JC1 is ready to receive data
    spi_send_one_float(tor_JC1_int); // send data
	while(CHECK1 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS1 = 1;                         // pull up CS pin of JC1

	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000000);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 is ready to receive data
	spi_send_one_float(tor_JC2_int); // send data
	while(CHECK2 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS2 = 1;                         // pull up CS pin of JC2
	
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000000);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 is ready to receive data
	spi_send_one_float(tor_JC3_int); // send data
	while(CHECK3 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//send three nominal torques to three controllers
void spi_send_nominal_torquelist(float *torque){
	float torque_JC1 = *torque++;
	float torque_JC2 = *torque++;	
	float torque_JC3 = *torque;		
	
	unsigned int tor_JC1_int = float_int_convert(torque_JC1);// convert from float to int for spi communication
	unsigned int tor_JC2_int = float_int_convert(torque_JC2);
	unsigned int tor_JC3_int = float_int_convert(torque_JC3);
	
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000002);   // send operating mode to JC1
	while(CHECK1 == 0){;}            // check if JC1 is ready to receive data
    spi_send_one_float(tor_JC1_int); // send data
	while(CHECK1 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS1 = 1;                         // pull up CS pin of JC1
	
    CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000002);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 is ready to receive data
    spi_send_one_float(tor_JC2_int); // send data
	while(CHECK2 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS2 = 1;                         // pull up CS pin of JC2
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000002);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 is ready to receive data
    spi_send_one_float(tor_JC3_int); // send data
	while(CHECK3 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//reset spring abs encoders
void spi_reset_spring_encoders(){
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000006);   // send operating mode to JC1  
	while(CHECK1 == 0){;}            // check if JC1 receives the command
	while(CHECK1 == 1){;}            // check if JC1 finishes the task
	CS1 = 1;                         // pull up CS pin of JC1
	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000006);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 receives the command
	while(CHECK2 == 1){;}            // check if JC2 finishes the task
	CS2 = 1;                         // pull up CS pin of JC2
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000006);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 receives the command
	while(CHECK3 == 1){;}            // check if JC3 finishes the task
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//reset incremental encoders
void spi_reset_inc_encoders(){
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000008);   // send operating mode to JC1  
	while(CHECK1 == 0){;}            // check if JC1 receives the command
	while(CHECK1 == 1){;}            // check if JC1 finishes the task
	CS1 = 1;                         // pull up CS pin of JC1
	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000008);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 receives the command
	while(CHECK2 == 1){;}            // check if JC2 finishes the task
	CS2 = 1;                         // pull up CS pin of JC2
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000008);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 receives the command
	while(CHECK3 == 1){;}            // check if JC3 finishes the task
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//read spring abs encoders
void spi_read_spring_encoders(float *encoderlist){
	CS1 = 0;                                    // lower CS pin of JC1
	spi_master_io_int(0x01000004);              // send operating mode to JC1 
    while(CHECK1 == 0){;}                       // check if JC1 is ready to send data
	float encoder_JC1_1 = spi_read_one_float(); // receive data
	CS1 = 1;                                    // pull up CS pin of JC1	
	
	CS2 = 0;                                    // lower CS pin of JC2
	spi_master_io_int(0x01000004);              // send operating mode to JC2 
    while(CHECK2 == 0){;}                       // check if JC2 is ready to send data
	float encoder_JC2_1 = spi_read_one_float(); // receive data
	CS2 = 1;                                    // pull up CS pin of JC2

	CS3 = 0;                                    // lower CS pin of JC3
	spi_master_io_int(0x01000004);              // send operating mode to JC3
    while(CHECK3 == 0){;}                       // check if JC3 is ready to send data
	float encoder_JC3_1 = spi_read_one_float(); // receive data
	CS3 = 1;                                    // pull up CS pin of JC3
	
	CS1 = 0;                                    // lower CS pin of JC1
	spi_master_io_int(0x01000005);              // send operating mode to JC1
    while(CHECK1 == 0){;}                       // check if JC1 is ready to send data
	float encoder_JC1_2 = spi_read_one_float(); // receive data
	CS1 = 1;                                    // pull up CS pin of JC1	
	
	CS2 = 0;                                    // lower CS pin of JC2
	spi_master_io_int(0x01000005);              // send operating mode to JC2
    while(CHECK2 == 0){;}                       // check if JC2 is ready to send data
	float encoder_JC2_2 = spi_read_one_float(); // receive data
	CS2 = 1;                                    // pull up CS pin of JC2

	CS3 = 0;                                    // lower CS pin of JC3
	spi_master_io_int(0x01000005);              // send operating mode to JC3
    while(CHECK3 == 0){;}                       // check if JC3 is ready to send data
	float encoder_JC3_2 = spi_read_one_float(); // receive data
	CS3 = 1;                                    // pull up CS pin of JC3
	
	*encoderlist++ = encoder_JC1_1;             // store data in a list
	*encoderlist++ = encoder_JC1_2;	
	*encoderlist++ = encoder_JC2_1;
	*encoderlist++ = encoder_JC2_2;
	*encoderlist++ = encoder_JC3_1;
	*encoderlist = encoder_JC3_2;
}

//read incremental encoders
void spi_read_inc_encoders(float *encoderlist){
	CS1 = 0;                                    // lower CS pin of JC1
	spi_master_io_int(0x01000009);              // send operating mode to JC1 
    while(CHECK1 == 0){;}                       // check if JC1 is ready to send data
	float encoder_JC1 = spi_read_one_float();   // receive data
	CS1 = 1;                                    // pull up CS pin of JC1	

	CS2 = 0;                                    // lower CS pin of JC2
	spi_master_io_int(0x01000009);              // send operating mode to JC2 
    while(CHECK2 == 0){;}                       // check if JC2 is ready to send data
	float encoder_JC2 = spi_read_one_float();   // receive data
	CS2 = 1;                                    // pull up CS pin of JC2

	CS3 = 0;                                    // lower CS pin of JC3
	spi_master_io_int(0x01000009);              // send operating mode to JC3
    while(CHECK3 == 0){;}                       // check if JC3 is ready to send data
	float encoder_JC3 = spi_read_one_float();   // receive data
	CS3 = 1;                                    // pull up CS pin of JC3

	*encoderlist++ = encoder_JC1;               // store data in a list
	*encoderlist++ = encoder_JC2;
	*encoderlist = encoder_JC3;
}

//send PWM
void spi_send_PWM(float *PWM_list){
	float PWM_JC1 = *PWM_list++;
	float PWM_JC2 = *PWM_list++;	
	float PWM_JC3 = *PWM_list;		
	
	unsigned int PWM_JC1_int = float_int_convert(PWM_JC1);// convert from float to int for spi communication
	unsigned int PWM_JC2_int = float_int_convert(PWM_JC2);
	unsigned int PWM_JC3_int = float_int_convert(PWM_JC3);
	
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000007);   // send operating mode to JC1 
	while(CHECK1 == 0){;}            // check if JC1 is ready to receive data
    spi_send_one_float(PWM_JC1_int); // send data
	while(CHECK1 == 1){;}            // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS1 = 1;                         // pull up CS pin of JC1
	
    CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000007);   // send operating mode to JC2 
	while(CHECK2 == 0){;}            // check if JC2 is ready to receive data
    spi_send_one_float(PWM_JC2_int); // send data
	while(CHECK2 == 1){;}            // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS2 = 1;                         // pull up CS pin of JC2
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000007);   // send operating mode to JC3 
	while(CHECK3 == 0){;}            // check if JC3 is ready to receive data
    spi_send_one_float(PWM_JC3_int); // send data
	while(CHECK3 == 1){;}            // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS3 = 1;                         // pull up CS pin of JC3
}

//set JCs to IDLE Mode
void spi_set_idle(){
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000010);   // send operating mode to JC1  
	while(CHECK1 == 0){;}            // check if JC1 receives the command
	while(CHECK1 == 1){;}            // check if JC1 finishes the task
	CS1 = 1;                         // pull up CS pin of JC1
	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000010);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 receives the command
	while(CHECK2 == 1){;}            // check if JC2 finishes the task
	CS2 = 1;                         // pull up CS pin of JC2
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000010);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 receives the command
	while(CHECK3 == 1){;}            // check if JC3 finishes the task
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//send three desired angles to three joint controllers
void spi_send_anglelist(float *angle){
	float angle_JC1 = *angle++;
	float angle_JC2 = *angle++;	
	float angle_JC3 = *angle;		
	
	unsigned int ang_JC1_int = float_int_convert(angle_JC1);// convert from float to int for spi communication
	unsigned int ang_JC2_int = float_int_convert(angle_JC2);
	unsigned int ang_JC3_int = float_int_convert(angle_JC3);
	
	
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000011);   // send operating mode to JC1
	while(CHECK1 == 0){;}            // check if JC1 is ready to receive data
    spi_send_one_float(ang_JC1_int); // send data
	while(CHECK1 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS1 = 1;                         // pull up CS pin of JC1

	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000011);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 is ready to receive data
	spi_send_one_float(ang_JC2_int); // send data
	while(CHECK2 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS2 = 1;                         // pull up CS pin of JC2
	
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000011);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 is ready to receive data
	spi_send_one_float(ang_JC3_int); // send data
	while(CHECK3 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//send counts for Mode3(TRACK)
void spi_send_counts(float count){
	unsigned int count_int = float_int_convert(count);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000012);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(count_int);     // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000012);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(count_int);     // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000012);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(count_int);     // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}

//send three desired angles to three joint controllers(in Mode3)
void spi_send_anglelist_Mode3(float *angle){
	float angle_JC1 = *angle++;
	float angle_JC2 = *angle++;	
	float angle_JC3 = *angle;		
	
	unsigned int ang_JC1_int = float_int_convert(angle_JC1);// convert from float to int for spi communication
	unsigned int ang_JC2_int = float_int_convert(angle_JC2);
	unsigned int ang_JC3_int = float_int_convert(angle_JC3);
	
	
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000013);   // send operating mode to JC1
	while(CHECK1 == 0){;}            // check if JC1 is ready to receive data
    spi_send_one_float(ang_JC1_int); // send data
	while(CHECK1 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS1 = 1;                         // pull up CS pin of JC1

	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000013);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 is ready to receive data
	spi_send_one_float(ang_JC2_int); // send data
	while(CHECK2 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS2 = 1;                         // pull up CS pin of JC2
	
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000013);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 is ready to receive data
	spi_send_one_float(ang_JC3_int); // send data
	while(CHECK3 == 1){;}            // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;} // delay for safety
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//set JCs to TRACK Mode
void spi_set_track(){
	CS1 = 0;                         // lower CS pin of JC1
	spi_master_io_int(0x01000014);   // send operating mode to JC1  
	while(CHECK1 == 0){;}            // check if JC1 receives the command
	while(CHECK1 == 1){;}            // check if JC1 finishes the task
	CS1 = 1;                         // pull up CS pin of JC1
	
	CS2 = 0;                         // lower CS pin of JC2
	spi_master_io_int(0x01000014);   // send operating mode to JC2
	while(CHECK2 == 0){;}            // check if JC2 receives the command
	while(CHECK2 == 1){;}            // check if JC2 finishes the task
	CS2 = 1;                         // pull up CS pin of JC2
	
	CS3 = 0;                         // lower CS pin of JC3
	spi_master_io_int(0x01000014);   // send operating mode to JC3
	while(CHECK3 == 0){;}            // check if JC3 receives the command
	while(CHECK3 == 1){;}            // check if JC3 finishes the task
	CS3 = 1;                         // pull up CS pin of JC3
	
}

//read incremental encoders
void spi_read_act_anglelist(float *anglelist){
	CS1 = 0;                                    // lower CS pin of JC1
	spi_master_io_int(0x01000015);              // send operating mode to JC1 
    while(CHECK1 == 0){;}                       // check if JC1 is ready to send data
	float encoder_JC1 = spi_read_one_float();   // receive data
	CS1 = 1;                                    // pull up CS pin of JC1	

	CS2 = 0;                                    // lower CS pin of JC2
	spi_master_io_int(0x01000015);              // send operating mode to JC2 
    while(CHECK2 == 0){;}                       // check if JC2 is ready to send data
	float encoder_JC2 = spi_read_one_float();   // receive data
	CS2 = 1;                                    // pull up CS pin of JC2

	CS3 = 0;                                    // lower CS pin of JC3
	spi_master_io_int(0x01000015);              // send operating mode to JC3
    while(CHECK3 == 0){;}                       // check if JC3 is ready to send data
	float encoder_JC3 = spi_read_one_float();   // receive data
	CS3 = 1;                                    // pull up CS pin of JC3

	*anglelist++ = encoder_JC1;               // store data in a list
	*anglelist++ = encoder_JC2;
	*anglelist = encoder_JC3;
}

//send Kp in current controller
void spi_send_Kp_c(float Kp_c){
	unsigned int Kp_c_int = float_int_convert(Kp_c);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000016);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(Kp_c_int);      // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000016);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(Kp_c_int);      // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000016);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(Kp_c_int);      // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}

//send Ki in current controller
void spi_send_Ki_c(float Ki_c){
	unsigned int Ki_c_int = float_int_convert(Ki_c);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000017);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(Ki_c_int);      // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000017);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(Ki_c_int);      // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000017);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(Ki_c_int);      // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}

//send Kp in position controller
void spi_send_Kp_p(float Kp_p){
	unsigned int Kp_p_int = float_int_convert(Kp_p);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000018);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(Kp_p_int);      // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000018);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(Kp_p_int);      // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000018);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(Kp_p_int);      // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}

//send Ki in position controller
void spi_send_Ki_p(float Ki_p){
	unsigned int Ki_p_int = float_int_convert(Ki_p);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000019);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(Ki_p_int);      // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000019);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(Ki_p_int);      // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000019);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(Ki_p_int);      // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}

//send Kd in position controller
void spi_send_Kd_p(float Kd_p){
	unsigned int Kd_p_int = float_int_convert(Kd_p);// convert from float to int for spi communication
	
	CS1 = 0;                           // lower CS pin of JC1
	spi_master_io_int(0x01000020);     // send operating mode to JC1
	while(CHECK1 == 0){;}              // check if JC1 is ready to receive data
    spi_send_one_float(Kd_p_int);      // send data
	while(CHECK1 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS1 = 1;                           // pull up CS pin of JC1
	
	CS2 = 0;                           // lower CS pin of JC2
	spi_master_io_int(0x01000020);     // send operating mode to JC2
	while(CHECK2 == 0){;}              // check if JC2 is ready to receive data
    spi_send_one_float(Kd_p_int);      // send data
	while(CHECK2 == 1){;}              // check if the data is received 
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS2 = 1;                           // pull up CS pin of JC2
	 
	CS3 = 0;                           // lower CS pin of JC3
	spi_master_io_int(0x01000020);     // send operating mode to JC3
	while(CHECK3 == 0){;}              // check if JC3 is ready to receive data
    spi_send_one_float(Kd_p_int);      // send data
	while(CHECK3 == 1){;}              // check if the data is received
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 300){;}   // delay for safety
	CS3 = 1;                           // pull up CS pin of JC3
}


//read one float number
float spi_read_one_float(){
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 200){;}                          // delay for safety	
	unsigned int number_int = spi_master_io_int(0x01000111);  // read the data (int form)
	float number = int_float_convert(number_int);             // convert int to float
	return number; 
}

//send one float number
void spi_send_one_float(int number){
	_CP0_SET_COUNT(0);
	while(_CP0_GET_COUNT() < 10){;}    // delay for safety
	spi_master_io_int(number);         // send the data (int form)
}


