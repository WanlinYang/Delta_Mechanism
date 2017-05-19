#include "spi_slave.h"    // spi communication with MMC
#include "NU32.h"         // NU32 head file
#include "SendControl.h"  // send control signal
#include "abs_encoder.h"  // spi communication with two spring absolute encoders




#define BUF_SIZE 200

static float Nominal_Torque = 0; // store nominal torque sent by MMC
static float Desired_Torque = 0; // store Desired_Torque
static float z_deg_1 = 0;        // 0 degree for abs_encoder1
static float z_deg_2 = 0;        // 0 degree for abs_encoder2
static float Stiffness = 0;      // store stiffness
static float PWM_counts = 0;     // store PWM count

void __ISR(_TIMER_2_VECTOR, IPL4SOFT) Controller(void){
	
	send_control_signal((int)PWM_counts);  // set PWM
	IFS0bits.T2IF = 0;                     // clear interrupt flag
	
}

int main(){
	__builtin_disable_interrupts();
	char buffer[BUF_SIZE];
	NU32_Startup();
	spi_slave_init();                 // initialize spi3 and communication with MMC
	abs_encoder_spi_master_init();    // initialize spi4 and communication with two spring encoders
	send_control_ini();               // PWM signal initialization(Timer2, Timer3 and OC1)
	unsigned int CN = 0;              // check mode number
	__builtin_enable_interrupts();
	while(1)
   {	
	__builtin_disable_interrupts();
 	CN = 0;                                // reset check mode number
	CN = spi_slave_io_int(0x00000026);     // receive check mode number(if ss pin is low)
    switch (CN){
	case 0x01000000:
	{//receive one desired torque
		Desired_Torque = spi_read_float();
		break;
    }
	case 0x01000001:
	{//send current torque   
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
	{//receive stiffness
		Stiffness = spi_read_float();
		break;
	}
	case 0x01000004:
	{//send data of spring encoder 1
		spi_send_float(abs_encoder1_deg(z_deg_1));
		break;
	}  
	case 0x01000005:
	{//send data of spring encoder 2
		spi_send_float(abs_encoder2_deg(z_deg_2));
		break;
	} 	
	case 0x01000006:
	{//reset two spring encoders	 
		CHECK = 1;
		z_deg_1 = abs_encoder1_absdeg();
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50000){;}
		z_deg_2 = abs_encoder2_absdeg();
		CHECK = 0;
		break;
	}  
	case 0x01000007:
	{//receive PWM_counts
		PWM_counts = spi_read_float();
		break;
	}  
    default:
    {//default(do nothing)   
        break;
    }	
   } 
   __builtin_enable_interrupts();	
  } 
}
