#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifndef INC_SOP_H_
#define INC_SOP_H_

#define MAX_PACK_VOLTAGE_V 600
#define STATIC_INTERNAL_RESISTANCE 1000 //placeholder value in milliohms
#define NOMINAL_CHARGE_CURRENT_MA 3000 //placeholder value in mA

float gt(uint16_t cell_temp_lowest);
float gz(uint32_t soc);
uint32_t SoPcalc(uint32_t soc, uint16_t cell_temp_lowest, float R0, float Inom);

#endif /* INC_SOP_H_ */