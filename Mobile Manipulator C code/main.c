#include <stdio.h>
#include "spi_master.h"      // spi communication with three joint controllers
#include "NU32.h"            // NU32.h
#include "Calibration.h"     // calibration
#include "gimbal_encoder.h"  // spi communication with three gimbal encoders
#include "Delta.h"


#define BUF_SIZE 200


static volatile float zero_deg_1 = 0;       // zero degree of gimbal encoder 1
static volatile float zero_deg_2 = 0;       // zero degree of gimbal encoder 2
static volatile float zero_deg_3 = 0;       // zero degree of gimbal encoder 3
static int position_list_count = 0;         // Mode3 position counts
static float desired_JC1_list[2000] = {0};  // store desired anglelist of JC1 in TRACK mode
static float desired_JC2_list[2000] = {0};  // store desired anglelist of JC2 in TRACK mode
static float desired_JC3_list[2000] = {0};  // store desired anglelist of JC2 in TRACK mode
static float R1 = 1,R2 = 1,L1 = 1,L2 = 1;   // store geometry parameters 

int main(){
	char buffer[BUF_SIZE];                       
	NU32_Startup();                  
	spi_master_init();               //initialize spi3 and communication with three joint controllers
	abs_encoder_spi_master_init();   //initialize spi4 and communication with three gimbal encoders
	while(1)
   {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
	switch (buffer[0]) 
	{
	  case 'a': // Calibration
      {
		float ntorquelist[3] = {0,0,0};  // define ntorquelist
		int i = 0;
		Calibration(ntorquelist);        // Calibration  
		for(i;i<3;i++)
		    {
			    sprintf(buffer,"%f\r\n",ntorquelist[i]);   // print the result
			    NU32_WriteUART3(buffer);
		    }
		break;
	  }
	  
	  case 'b':// Read gimbal encoders
	  {
		float f = abs_encoder1_deg(zero_deg_1);  // read gimbal encoder 1  
		sprintf(buffer,"%.3f\r\n",f);            // print the result
		NU32_WriteUART3(buffer);
		f = abs_encoder2_deg(zero_deg_2);        // read gimbal encoder 2
		sprintf(buffer,"%.3f\r\n",f);            // print the result
		NU32_WriteUART3(buffer);		
		f = abs_encoder3_deg(zero_deg_3);        // read gimbal encoder 3
		sprintf(buffer,"%.3f\r\n",f);            // print the result
		NU32_WriteUART3(buffer);
		break;   
	  }  
	  
	  case 'c':// Reset three gimbal encoders
	  {
		zero_deg_1 = abs_encoder1_absdeg();      // set zero_deg_1 as a reference
		float f = abs_encoder1_deg(zero_deg_1);  // read gimbal encoder 1 
		sprintf(buffer,"%.3f\r\n",f);            // print the result
		NU32_WriteUART3(buffer);
		zero_deg_2 = abs_encoder2_absdeg();      // set zero_deg_2 as a reference
		f = abs_encoder2_deg(zero_deg_2);        // read gimbal encoder 2 
		sprintf(buffer,"%.3f\r\n",f);            // print the result
		NU32_WriteUART3(buffer);		
		zero_deg_3 = abs_encoder3_absdeg();      // set zero_deg_3 as a reference
		f = abs_encoder3_deg(zero_deg_3);        // read gimbal encoder 3 
		sprintf(buffer,"%.3f\r\n",f);            // print the result
		NU32_WriteUART3(buffer);		
		break;    
	  }	
	  
	  case 'd':// Send desired torques to 3 JCs
	  {
		float torquelist[3];                     
        int i = 0;
		for(i;i < 3;i++){
            NU32_ReadUART3(buffer,BUF_SIZE);      // read three desired torques from user
	        sscanf(buffer,"%f",&torquelist[i]);
		}
		spi_send_torquelist(torquelist);          // send torques to each JC
		break;    
	  }	  
	  case 'e':// Receive current torques from 3 JCs
	  {
		float torquelist[3] = {0,0,0};            
		int i = 0;
		spi_read_torquelist(torquelist);                // read current torques from JCs
		for(i;i<3;i++)
		    {
			    sprintf(buffer,"%f\r\n",torquelist[i]); // print the results
			    NU32_WriteUART3(buffer);
		    }
		break;    
	  }	
	  case 'f':// Send Stiffness to 3 JCs
	  {
		float Stiffness;
		NU32_ReadUART3(buffer,BUF_SIZE);      // read stiffness from user
	    sscanf(buffer,"%f",&Stiffness);
        spi_send_Stiffness(Stiffness);        // send stiffness to each JC 
		break;    
	  }
      case 'g':// Reset spring abs encoders
	  {
        spi_reset_spring_encoders();          // reset spring abs encoders
		break;    
	  }		  
      case 'h':// Read spring abs encoders
	  {
        float encoderlist[6] = {0,0,0,0,0,0};
		spi_read_spring_encoders(encoderlist);           // read spring abs encoders from JCs
		int i = 0;
		for(i;i<6;i++)
		    {
			    sprintf(buffer,"%f\r\n",encoderlist[i]); // print the results
			    NU32_WriteUART3(buffer);
		    }
		break;    
	  }
	  case 'i':// Read geometry parameters 
	  {
		NU32_ReadUART3(buffer,BUF_SIZE);   // read R1 from user
	    sscanf(buffer,"%f",&R1);
		NU32_ReadUART3(buffer,BUF_SIZE);   // read R2 from user
	    sscanf(buffer,"%f",&R2);
		NU32_ReadUART3(buffer,BUF_SIZE);   // read L1 from user
	    sscanf(buffer,"%f",&L1);		
		NU32_ReadUART3(buffer,BUF_SIZE);   // read L2 from user
	    sscanf(buffer,"%f",&L2); 
		break;  
	  }
	  case 'j':// Send PWM
	  {
        float PWM_list[3];
		int i = 0;
		for(i;i < 3;i++){
            NU32_ReadUART3(buffer,BUF_SIZE);   // read PWM for each JC from user
	        sscanf(buffer,"%f",&PWM_list[i]);
		}
		spi_send_PWM(PWM_list);                // send PWM
		break;    
	  }	
	  case 'k':// Reset incremental encoders
	  {
        spi_reset_inc_encoders();          // reset incremental encoders
		break;    
	  }		
	  case 'l':// Read incremental encoders
	  {
        float encoderlist[3] = {0,0,0};
		spi_read_inc_encoders(encoderlist);           // read spring abs encoders from JCs
		int i = 0;
		for(i;i<3;i++)
		    {
			    sprintf(buffer,"%f\r\n",encoderlist[i]); // print the results
			    NU32_WriteUART3(buffer);
		    }
		break;     
	  }	
	  case 'u':// Set control gains
	  {
		float Kp_c,Ki_c,Kp_p,Ki_p,Kd_p;
		NU32_ReadUART3(buffer,BUF_SIZE);   // read Kp in current controller from user
	    sscanf(buffer,"%f",&Kp_c);
		NU32_ReadUART3(buffer,BUF_SIZE);   // read Ki in current controller from user
	    sscanf(buffer,"%f",&Ki_c);
		NU32_ReadUART3(buffer,BUF_SIZE);   // read Kp in position controller from user
	    sscanf(buffer,"%f",&Kp_p);		
		NU32_ReadUART3(buffer,BUF_SIZE);   // read Ki in position controller from user
	    sscanf(buffer,"%f",&Ki_p);   
		NU32_ReadUART3(buffer,BUF_SIZE);   // read Ki in position controller from user
	    sscanf(buffer,"%f",&Kd_p);
		spi_send_Kp_c(Kp_c);
		spi_send_Ki_c(Ki_c);
		spi_send_Kp_p(Kp_p);
		spi_send_Ki_p(Ki_p);
		spi_send_Kd_p(Kd_p);
		break;
	  }
	  case 'p':// Set JCs to IDLE Mode
	  {
        spi_set_idle();         // Set JCs to IDLE Mode
		break;    
	  }
	  case 'o':// move end-effect to an certain position
	  {
		float angle_list[3];
		float x,y,z;
		NU32_ReadUART3(buffer,BUF_SIZE);   // read R2 from user
	    sscanf(buffer,"%f",&x);
		NU32_ReadUART3(buffer,BUF_SIZE);   // read R2 from user
	    sscanf(buffer,"%f",&y);
		NU32_ReadUART3(buffer,BUF_SIZE);   // read R2 from user
	    sscanf(buffer,"%f",&z); 
		
		DeltaIkin(R1,R2,L1,L2,x,y,z,angle_list);
		
		int i = 0;
		for(i;i < 3;i++){
			angle_list[i] = angle_list[i]*180/PI;
		}
		spi_send_anglelist(angle_list);        // Send desired angle to JCs
		break;
	  }	
	  case 'n':// load cubic trajectory
	  {
	    int count = 0;
		int i = 0;
		int j = 0;
		float angle_list[3];
		float desired_position_xlist[2000];
		float desired_position_ylist[2000];
		float desired_position_zlist[2000];
		NU32_ReadUART3(buffer,BUF_SIZE);
		sscanf(buffer,"%d",&count);
		spi_send_counts(count);
		
		position_list_count = count;
		
		for(i=0;i < count;i++){
            NU32_ReadUART3(buffer,BUF_SIZE);
			sscanf(buffer,"%f",&desired_position_xlist[i]);
		}
		for(i=0;i < count;i++){
            NU32_ReadUART3(buffer,BUF_SIZE);
			sscanf(buffer,"%f",&desired_position_ylist[i]);
		}
		for(i=0;i < count;i++){
            NU32_ReadUART3(buffer,BUF_SIZE);
			sscanf(buffer,"%f",&desired_position_zlist[i]);
		}
		for(i=0;i < count;i++){
			DeltaIkin(R1,R2,L1,L2,desired_position_xlist[i],desired_position_ylist[i],desired_position_zlist[i],angle_list);
			for(j=0;j < 3;j++){
				angle_list[j] = angle_list[j]*180/PI;
			}
			desired_JC1_list[i] = angle_list[0];
			desired_JC2_list[i] = angle_list[1];
			desired_JC3_list[i] = angle_list[2];
			
			spi_send_anglelist_Mode3(angle_list);
		}
		
		break;
	  }	
	  case 'y':// Set JCs to TRACK Mode
	  {
        spi_set_track();         // Set JCs to IDLE Mode
		break;    
	  }	
	  
	  case 'r':// Read actual anglelist data from JCs(Mode3)
	  {
		int i = 0;
		int j = 0;
		float anglelist[3];
		sprintf(buffer,"%d\r\n",position_list_count);
		NU32_WriteUART3(buffer);
		for(i=0;i < position_list_count;i++)
		{
			spi_read_act_anglelist(anglelist);
			sprintf(buffer,"%f %f %f %f %f %f\r\n",anglelist[0],anglelist[1],anglelist[2],desired_JC1_list[i],desired_JC2_list[i],desired_JC3_list[i]); // print the results
			NU32_WriteUART3(buffer);
		}
		break;    
	  }
	  
	  default:// Default
      {
                    // do nothing
        break;  
      }	
	}
   }
} 


