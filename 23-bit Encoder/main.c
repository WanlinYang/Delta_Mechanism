#include<stdio.h>
#include"op_encoder.h"

#define BUF_SIZE 200

static volatile float zero_deg = 0;       // global variable indicating zero degree of encoder

int main(){
	char buffer[BUF_SIZE];
	NU32_Startup();
	op_encoder_spi_master_init();
	NU32_WriteUART3("initialize\r\n");
	
	while(1){
		NU32_ReadUART3(buffer,BUF_SIZE);  // we expect the next character to be a menu command
		switch(buffer[0]){
			case 'l':		// read binary output from encoder
			{
				uint32_t read_encoder = op_encoder_hex();
				sprintf(buffer,"%0x\r\n",read_encoder);
				NU32_WriteUART3(buffer);
				break;
			}
			case 'k':       // read absolute degree output from encoder
			{
				float deg = op_encoder_deg(zero_deg);
				sprintf(buffer,"%8f\r\n",deg);
				NU32_WriteUART3(buffer);
				break;
			}
			case 'r':       // reset zero degree
			{
				zero_deg = op_encoder_absdeg();
				NU32_WriteUART3("zero degree has been reset\r\n");
				break;
			}
			case 'c':       // continuously measurement
			{
				int i = 0, j = 0;
				float degrees[10000];
				for(i=0; i<7700; i++){   // up to 7700 times per second
					degrees[i] = op_encoder_deg(zero_deg);
					j = _CP0_GET_COUNT();
					_CP0_SET_COUNT(0);
					while(_CP0_GET_COUNT()<300) {;}   // 7.5us
				}
				sprintf(buffer,"j=%d\r\n",j);    // j = 4797 ==> 120us ==> 120+7.5us ~= 130us ==> 7700 times/sec
				NU32_WriteUART3(buffer);
			}
		}
	}
} 








