#include "Calibration.h" 


void Calibration(float *ntorquelist){
	char buffer[BUF_SIZE];
	sprintf(buffer,"Please move manipulator to home position.\r\n");
	NU32_WriteUART3(buffer);
	NU32_ReadUART3(buffer,BUF_SIZE);
	float torquelist_home[3];                   //define torquelist in home position 
	spi_read_torquelist(torquelist_home);       //read home position torque
	
	
	sprintf(buffer,"Please load the article.\r\n");
	NU32_WriteUART3(buffer);
	NU32_ReadUART3(buffer,BUF_SIZE);
	float torquelist_article[3];               //define torquelist in article position
	spi_read_torquelist(torquelist_article);   //read article position torque
	
	
	mr_MinusVectors(torquelist_article,torquelist_home,3,ntorquelist);   //calculate nominal torquelist
	spi_send_nominal_torquelist(ntorquelist);                            //send nominal torquelist to JCs	
}