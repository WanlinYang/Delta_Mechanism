#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "NU32.h" 
#include "spi_master.h"
#include "Delta.h"

#define BUF_SIZE 200

void Calibration(float *ntorquelist);

#endif//CALIBRATION_H