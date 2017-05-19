#include "float_int.h"

unsigned int float_int_convert(float f){
	uint32_t *d = (uint32_t*)&f;    // temporary pointer of type uint32_t
	unsigned d_value = *d;          // value pointed by the pointer
	return d_value;
}

float int_float_convert(unsigned int d_value){
	float f;
	uint32_t *q = (uint32_t*)&f;    // temporary pointer pointing f
	*q = d_value;
	return f;
}












