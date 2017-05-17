#include<stdio.h>
#include"spi_master.h"       // spi communication with three joint controllers
#include "NU32.h"            // NU32.h
#include "Calibration.h"     // calibration
#include "gimbal_encoder.h"  // spi communication with three gimbal encoders


#define BUF_SIZE 200

static volatile float zero_deg_1 = 0;  // zero degree of gimbal encoder 1
static volatile float zero_deg_2 = 0;  // zero degree of gimbal encoder 2
static volatile float zero_deg_3 = 0;  // zero degree of gimbal encoder 3


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
	  case 'j':// Send PWM
	  {
        float PWM_list[3];
		
		NU32_ReadUART3(buffer,BUF_SIZE);   // read PWM for each JC from user
		sscanf(buffer,"%f  %f  %f",&PWM_list[0],&PWM_list[1],&PWM_list[2]);
		spi_send_PWM(PWM_list);                // send PWM
		
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


