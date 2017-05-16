#include<stdio.h>
#include"spi_master.h"       //spi communication with three joint controllers
#include "NU32.h"            //NU32.h
#include "Calibration.h"     //for calibration
#include "gimbal_encoder.h"  //spi communication with three gimbal encoders


#define BUF_SIZE 200

static volatile float zero_deg_1 = 0;  //zero degree of gimbal angle 1
static volatile float zero_deg_2 = 0;  //zero degree of gimbal angle 2
static volatile float zero_deg_3 = 0;  //zero degree of gimbal angle 3


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
		float ntorquelist[3] = {0,0,0};
		int i = 0;
		Calibration(ntorquelist);
		for(i;i<3;i++)
		    {
			    sprintf(buffer,"%f\r\n",ntorquelist[i]); 
			    NU32_WriteUART3(buffer);
		    }
		break;
	  }
	  
	  case 'b'://Read gimbal encoders
	  {
		float f = abs_encoder1_deg(zero_deg_1);
		sprintf(buffer,"%.3f\r\n",f);
		NU32_WriteUART3(buffer);
		f = abs_encoder2_deg(zero_deg_2);
		sprintf(buffer,"%.3f\r\n",f);
		NU32_WriteUART3(buffer);		
		f = abs_encoder3_deg(zero_deg_3);
		sprintf(buffer,"%.3f\r\n",f);
		NU32_WriteUART3(buffer);
		break;   
	  }  
	  
	  case 'c'://Reset three gimbal encoders
	  {
		zero_deg_1 = abs_encoder1_absdeg();
		float f = abs_encoder1_deg(zero_deg_1);
		sprintf(buffer,"%.3f\r\n",f);
		NU32_WriteUART3(buffer);
		zero_deg_2 = abs_encoder2_absdeg();
		f = abs_encoder2_deg(zero_deg_2);
		sprintf(buffer,"%.3f\r\n",f);
		NU32_WriteUART3(buffer);		
		zero_deg_3 = abs_encoder3_absdeg();
		f = abs_encoder3_deg(zero_deg_3);
		sprintf(buffer,"%.3f\r\n",f);
		NU32_WriteUART3(buffer);		
		break;    
	  }	
	  
	  case 'd'://send desired torques to 3 JCs
	  {
		float torquelist[3] = {2.5123541,8.6235412,10.2154235};
        spi_send_torquelist(torquelist);
		int i = 0;
		for(i;i<3;i++)
		    {
			    sprintf(buffer,"%f\r\n",torquelist[i]); 
			    NU32_WriteUART3(buffer);
		    }
		break;    
	  }	  
	  case 'e'://Receive current torques from 3 JCs
	  {
		float torquelist[3] = {0,0,0};
		int i = 0;
		spi_read_torquelist(torquelist);
		for(i;i<3;i++)
		    {
			    sprintf(buffer,"%f\r\n",torquelist[i]); 
			    NU32_WriteUART3(buffer);
		    }
		break;    
	  }	
	  case 'f'://Receive current torques from 3 JCs
	  {
		float Stiffness = 0.5;
        spi_send_Stiffness(Stiffness);
		break;    
	  }
      case 'g'://reset spring abs encoders
	  {
        spi_reset_spring_encoders();
		break;    
	  }		  
      case 'h'://read spring abs encoders
	  {
        float encoderlist[6] = {0,0,0,0,0,0};
		spi_read_spring_encoders(encoderlist);
		int i = 0;
		for(i;i<6;i++)
		    {
			    sprintf(buffer,"%f\r\n",encoderlist[i]); 
			    NU32_WriteUART3(buffer);
		    }
		break;    
	  }	
	  case 'j'://read spring abs encoders
	  {
        float PWM_list[3];
		int i = 0;
		for(i;i < 3;i++){
            NU32_ReadUART3(buffer,BUF_SIZE);
	        sscanf(buffer,"%f",&PWM_list[i]);
		}
		spi_send_PWM(PWM_list);
		break;    
	  }	
	  default:
      {
                    
        break;
      }	
	}
   }
} 


