#include "spi_slave.h"  // spi communication
#include "NU32.h"  // NU32 head file
#include "SendControl.h"  // send control signal
#include "abs_encoder.h"




#define BUF_SIZE 200

static float Nominal_Torque = 0; // store nominal torque sent by MMC
static float Desired_Torque = 0; // store Desired_Torque
static float z_deg_1 = 0;        // 0 degree for abs_encoder1
static float z_deg_2 = 0;        // 0 degree for abs_encoder2
static float Stiffness = 0;      // store stiffness




int main(){
	char buffer[BUF_SIZE];
	NU32_Startup();
	spi_slave_init();
	abs_encoder_spi_master_init();
	unsigned int CN = 0;
    NU32_LED1 = 1;
    NU32_LED2 = 1;
	while(1)
   {	
	//for communication
	//for each case send a different buffer(for following operations)
	//if(LATEbits.LATE2 == 0){
	CN = 0;
	CN = spi_slave_io_int(0x00000026);
    switch (CN){
	case 0x01000000:
	{//receive one desired torque
		Desired_Torque = spi_read_float();
		break;
    }
	case 0x01000001:
	{   
	    //float temp = joint_torque
		float temp = joint_torque(Stiffness,z_deg_1,z_deg_2);
		spi_send_float(temp);
		break;
    }
    case 0x01000002:
	{//receive one nominal torque
	    Nominal_Torque = spi_read_float();
		break;
	}
	
    case 0x01000003:
	{
		Stiffness = spi_read_float();
		break;
	}
	case 0x01000004:
	{	
		spi_send_float(abs_encoder1_deg(z_deg_1));
		break;
	}  
	case 0x01000005:
	{	
		spi_send_float(abs_encoder2_deg(z_deg_2));
		break;
	} 	
	case 0x01000006:
	{	
        NU32_LED1 = !NU32_LED1; 
		NU32_LED2 = !NU32_LED2; 
		CHECK = 1;
		z_deg_1 = abs_encoder1_absdeg();
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50000){;}
		z_deg_2 = abs_encoder2_absdeg();
		CHECK = 0;
		break;
	}  
    default:
    {          
        break;
    }	
   }
/* 	switch (buffer[0]) {
      case 'a': 
      {  
		
		break;
	  }
	  case 'b'://send control signal(not sure it is a function or a case)
	  {
		//send control signals here(probably in an interrupy)
		
		break;
	  } 
	  case 'c':
	  {
		//send control signals here(probably in an interrupy)
		break;
	  }  	  
   } */
	
} 
}
