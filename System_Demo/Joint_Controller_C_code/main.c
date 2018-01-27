#include "spi_slave.h"    // spi communication with MMC
#include "NU32.h"         // NU32 head file
#include "SendControl.h"  // send control signal
//#include "abs_encoder.h"  // spi communication with two spring absolute encoders
#include "encoder.h"      // spi communication with motor incremental encoder



#define BUF_SIZE 200


static int Mode = 0;             // operating mode(0-IDLE,1-PWM,2-HOLD,3-TRACK)
static float Nominal_Torque = 0; // store nominal torque sent by MMC
static float Desired_Torque = 0; // store Desired_Torque
static float z_deg_1 = 0;        // 0 degree for abs_encoder1
static float z_deg_2 = 0;        // 0 degree for abs_encoder2
static float Stiffness = 0;      // store stiffness
static float PWM_counts = 0;     // store PWM count
static  float Eint_current = 0;  // current(PI) controller integral item
static  float Kp_current = 50;   // current(PI) controller-Kp
static  float Ki_current = 5;    // current(PI) controller-Ki
static  float Eint_position = 0; // position(PID) controller integral item
static  float Kp_position = 5;   // position(PID) controller-Kp
static  float Ki_position = 0.1;   // position(PID) controller-Ki
static  float Kd_position = 200;   // position(PID) controller-Kd
static  float Error_position_prev = 0;      // position(PID) controller previous error(to calculate Edot)
static  float Desired_Position = 0;         // store Desired_Position for Mode 2
static  float Desired_Position_List[2000];  // store Desired_Position_List for Mode 3
static  int Desired_Position_Count = 0;     // store number of points in Mode 3
static  int Actual_Position_Count = 0;      // count for trajectory
static  float Position_Output_List[2000];   // store actual positions to output
static  float Desired_Current = 0;          // store the current(mA) calculated in position controller and it will be used in current controller


// position controller (Timer 4 interrupt at )
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller_position(void){
	static float Error_position = 0;  // position error
	static float u_position = 0;      // control signal u
	static float Edot_position = 0;   // edot 
	if (Mode == 2){   // HOLD Mode
		Error_position = Desired_Position - encoder_angle();
		Eint_position = Eint_position + Error_position;
		if(Eint_position >= 1000){                    // anti wind up
			Eint_position = 1000;
		}
		if(Eint_position <= -1000){         
			Eint_position = -1000;
		}
		Edot_position = Error_position - Error_position_prev;  // calculate edot
		u_position = Kp_position*Error_position + Ki_position*Eint_position + Kd_position*Edot_position; // u = Kp*err + Ki*eint + Kd*edot
		Error_position_prev = Error_position;   // store Error_position for next time
		Desired_Current = u_position;           // calculate the desired current
	}
	
 	if (Mode == 3){  // TRACK Mode
		Desired_Position = Desired_Position_List[Actual_Position_Count]; // define Desired_Position from the array
		Position_Output_List[Actual_Position_Count] = encoder_angle();   // read current angle and store in an array
		Error_position = Desired_Position - Position_Output_List[Actual_Position_Count];  // calculate the error
		
		Eint_position = Eint_position + Error_position;
		if(Eint_position >= 1000){                    // anti wind up
			Eint_position = 1000;
		}
		if(Eint_position <= -1000){
			Eint_position = -1000;
		}
		Edot_position = Error_position - Error_position_prev; // calculate edot
		u_position = Kp_position*Error_position + Ki_position*Eint_position + Kd_position*Edot_position; // u = Kp*err + Ki*eint + Kd*edot
		Error_position_prev = Error_position;   // store Error_position for next time 
		Desired_Current = u_position;           // calculate the desired current 
		Actual_Position_Count = Actual_Position_Count + 1; 
		if (Actual_Position_Count == Desired_Position_Count){ // when TRACK mode is finished 	
			  Error_position_prev = 0;     // return to zero for HOLD mode                 
		      Eint_position = 0;
		      Mode = 2;
		      Eint_current = 0;
		}
	} 
	IFS0bits.T4IF = 0;                     // clear flag status
	
}




void __ISR(_TIMER_2_VECTOR, IPL4SOFT) Controller(void){
	
	switch (Mode) {
	   case 0:  // IDLE Mode
      {
        OC1RS = 0; 
		break;
	  }
	   case 1:  // PWM Mode
	  {   
	    send_control_signal((int)PWM_counts);  // set PWM
		break;
	  }
	   case 2:  // HOLD Mode
	   case 3:  // TRACK Mode
	   {
	        static int Current_Ref = 0;        // current reference 
		    static float Current_Act = 0;      // actual current 
		    static float Error_current = 0;    // current error
		    static float u_current = 0;		   // control signal
			
			if (Desired_Current > 1000)        // limit the desired current -1000 to 1000
			{
				Current_Ref = 1000;
	        }
			else if(Desired_Current < -1000)
			{
				Current_Ref = -1000;
	        }
			else if(Desired_Current >= -1000 && Desired_Current <= 1000)
			{
				Current_Ref = Desired_Current;
	        } 

			Current_Act = adc_sample_convert(12)*1.927-977.39;  // read actual current 
	        Error_current = Current_Ref - Current_Act;          // calculate error
	        Eint_current = Eint_current + Error_current;        // calculate eint
			
			if (Eint_current >=500)    // anti wind up
			{
				Eint_current = 500;
			}
			if(Eint_current <=-500)
			{
				Eint_current = -500;
			}	
			
	        u_current = Kp_current*Error_current + Ki_current*Eint_current;   // u = Kp*error + Ki*eint
			
			if (u_current > 6000)         //limit control signal -6000 to 6000
			{
				u_current = 6000;
	        }
			else if (u_current < -6000)
			{
				u_current = -6000;
	        }
			  
			if (u_current >= 0)                     // set duty cycle of PWM and motor direction    
			{
				OC1RS = (int)(0.66667*u_current); 
				LATDbits.LATD6 = 0;
			}
			else
			{
				OC1RS = (int)(-0.66667*u_current); 
				LATDbits.LATD6 = 1;
			}
			break;
		}

	}
	IFS0bits.T2IF = 0;                     // clear interrupt flag
	
}

int main(){
	__builtin_disable_interrupts();
	char buffer[BUF_SIZE];
	Mode = 0;                         // IDLE Mode
	NU32_Startup();
	spi_slave_init();                 // initialize spi3 and communication with MMC
	/* abs_encoder_spi_master_init();    // initialize spi4 and communication with two spring encoders */
	encoder_init();                   // incremental encoder initialization
// initialize ADC
	AD1PCFGbits.PCFG12 = 0;
    AD1CON3bits.ADCS = 2;
    AD1CON1bits.ADON = 1; 
// end
	send_control_ini();               // PWM signal initialization(Timer2, Timer3 and OC1)
	unsigned int CN = 0;              // check mode number
	__builtin_enable_interrupts();
	while(1)
   {	
 	CN = 0;                                // reset check mode number
	CN = spi_slave_io_int(0x00000026);     // receive check mode number(if ss pin is low)
    switch (CN){
	case 0x01000000:
	{//receive one desired torque
		__builtin_disable_interrupts();      // disable interrupt
		Desired_Torque = spi_read_float();   // read data
		__builtin_enable_interrupts();       // enable interrupt
		break;
    }
	case 0x01000001:
	{//send current torque   
		__builtin_disable_interrupts();      // disable interrupt
		float temp = joint_torque(Stiffness,z_deg_1,z_deg_2);  // calculate current torque
		spi_send_float(temp);                // send data
		__builtin_enable_interrupts();       // enable interrupt
		break;
    }
    case 0x01000002:
	{//receive one nominal torque
		__builtin_disable_interrupts();      // disable interrupt
	    Nominal_Torque = spi_read_float();   // read data
		__builtin_enable_interrupts();	     // enable interrupt	
		break;
	}
	
    case 0x01000003:
	{//receive stiffness
		__builtin_disable_interrupts();      // disable interrupt
		Stiffness = spi_read_float();        // read data
		__builtin_enable_interrupts();		 // enable interrupt	
		break;
	}
/* 	case 0x01000004:
	{//send data of spring encoder 1
		__builtin_disable_interrupts();             // disable interrupt
		spi_send_float(abs_encoder1_deg(z_deg_1));  // send data
		__builtin_enable_interrupts();		        // enable interrupt
		break;
	}  
	case 0x01000005:
	{//send data of spring encoder 2
		__builtin_disable_interrupts();             // disable interrupt		
		spi_send_float(abs_encoder2_deg(z_deg_2));  // send data
		__builtin_enable_interrupts();		        // enable interrupt		
		break;
	} 	
	case 0x01000006:
	{//reset two spring encoders	 
		CHECK = 1;                                  // send the signal that JC is ready to finish the task
		z_deg_1 = abs_encoder1_absdeg();            // reset encoder 1
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50000){;}          // delay to make sure the signal is detected by MMC
		z_deg_2 = abs_encoder2_absdeg();            // reset encoder 2
		CHECK = 0;                                  // send the signal that JC has finished the task
		break;
	}   */
	case 0x01000007:
	{//receive PWM_counts
		__builtin_disable_interrupts();		// disable interrupt
		PWM_counts = spi_read_float();      // read data
		Mode = 1;                           // change mode into PWM
		__builtin_enable_interrupts();		// enable interrupt		
		break;
	}  
	case 0x01000008:
	{//reset incremental encoder
		CHECK = 1;                             // send the signal that JC is ready to finish the task
		encoder_reset();                       // reset encoder
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50000){;}     // delay to make sure the signal is detected by MMC
		CHECK = 0;                             // send the signal that JC has finished the task
		break;
	}  
	case 0x01000009:
	{//send data of incremental encoder
		__builtin_disable_interrupts();	       // disable interrupt		
		spi_send_float(encoder_angle());       // send data
		__builtin_enable_interrupts();	       // enable interrupt		
		break;
	}  
    case 0x01000010:
	{//set Mode to IDLE	
		CHECK = 1;                           // send the signal that JC is ready to finish the task
		Mode = 0;                            // change mode into IDLE
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50000){;}   // delay to make sure the signal is detected by MMC
		CHECK = 0; 	                         // send the signal that JC has finished the task
		break;
	} 
	case 0x01000011:
	{//set Mode to HOLD
		__builtin_disable_interrupts();			// disable interrupt	
		Desired_Position = spi_read_float();    // read data
		__builtin_enable_interrupts();          // enable interrupt
		Eint_current = 0;                       
		Eint_position = 0;
		Error_position_prev = 0;
		Mode = 2;
			
		break;
	} 
	case 0x01000012:
	{//receive global_position_count
		__builtin_disable_interrupts();		            // disable interrupt
		Desired_Position_Count = (int)spi_read_float(); // read data(int form)
		__builtin_enable_interrupts();			        // enable interrupt
		break;
	} 
	case 0x01000013:
	{//receive anglelist in TRACK Mode
		static int i_check = 0;
		if (i_check < Desired_Position_Count){
			__builtin_disable_interrupts();                    // disable interrupt
			Desired_Position_List[i_check] = spi_read_float(); // read data
			__builtin_enable_interrupts();		               // enable interrupt		
			i_check = i_check + 1;
		}
	    if (i_check == Desired_Position_Count){
			i_check = 0;
		}

		break;
	}
	case 0x01000014:
	{//set Mode to TRACK
		CHECK = 1;                         // send the signal that JC is ready to finish the task
		Actual_Position_Count = 0;         // count in TRACK mode
		Error_position_prev = 0;
		Eint_position = 0;
		Eint_current = 0;
		Mode = 3;
		_CP0_SET_COUNT(0);
		while(_CP0_GET_COUNT() < 50000){;} // delay to make sure the signal is detected by MMC
		CHECK = 0; 	                       // send the signal that JC has finished the task
		break;
	}
	case 0x01000015:
	{//send anglelist in TRACK Mode
		static int i_o_check = 0;
		if (i_o_check < Desired_Position_Count){
			__builtin_disable_interrupts();                  // disable interrupt
			spi_send_float(Position_Output_List[i_o_check]); // send data
			__builtin_enable_interrupts();                   // enable interrupt
			i_o_check = i_o_check + 1;
		}
	    if (i_o_check == Desired_Position_Count){
			i_o_check = 0;
		}
		break;
	}
	case 0x01000016:
	{//read Kp in current controller
		__builtin_disable_interrupts();  // disable interrupt
		Kp_current = spi_read_float();   // read data
		__builtin_enable_interrupts();   // enable interrupt
		break;
	}
	case 0x01000017:
	{//read Ki in current controller
		__builtin_disable_interrupts();  // disable interrupt
		Ki_current = spi_read_float();   // read data
		__builtin_enable_interrupts();   // enable interrupt
		break;
	}
	case 0x01000018:
	{//read Kp in position controller
		__builtin_disable_interrupts();  // disable interrupt
		Kp_position = spi_read_float();  // read data
		__builtin_enable_interrupts();   // enable interrupt
		break;
	}
	case 0x01000019:
	{//read Ki in position controller
		__builtin_disable_interrupts();  // disable interrupt
		Ki_position = spi_read_float();  // read data
		__builtin_enable_interrupts();   // enable interrupt
		break;
	}
	case 0x01000020:
	{//read Kd in position controller
		__builtin_disable_interrupts();  // disable interrupt
		Kd_position = spi_read_float();  // read data
		__builtin_enable_interrupts();   // enable interrupt
		break;
	}
    default:
    {//default(do nothing)   
        break;
    }	
   } 	
  } 
}
