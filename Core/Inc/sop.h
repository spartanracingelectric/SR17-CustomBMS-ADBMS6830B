#ifndef INC_SOP_H_
#define INC_SOP_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#define MAX_PACK_VOLTAGE_MV 600000 //comp value in mV
#define MAX_CHARGER_CURRENT_MA 14000 //charger current maximum in mA
#define INT_RES_AVG_45C_UO 2540 //placeholder value in microohms
#define NOMINAL_CHARGE_CURRENT_MA 7500 //value in mA from datasheet (for full pack); could go to 30 A
#define RES_TEMP_SLOPE 6 // placeholder in permille per C
#define COLD_STOP_CHARGE_TEMP_C 15 //placeholder estimate in C (maybe 0?)
#define COLD_FULL_CHARGE_TEMP_C 20 //placeholder estimate in C (unclear)
#define HOT_FULL_CHARGE_TEMP_C 45 //placeholder estimate in C
#define HOT_STOP_CHARGE_TEMP_C 60 //in C
#define LOW_RAMP_SOC 20 //unitless, permille
#define LOW_FULL_SOC 50 //unitless, permille
#define HIGH_RAMP_SOC 900 //unitless, permille
#define HIGH_EDGE_SOC 970 //unitless, permille
#define NUM_CELLS_2 14 //might differ with new pack, couldn't access main.h variable
#define CHARGER_ENABLED 1 //placeholder until bhuvanesh defines this elsewhere

uint32_t SOP_gt(uint16_t cell_temp_lowest);
uint32_t SOP_gz(uint32_t soc);
uint32_t Vsaf(uint16_t cell_temp_lowest);
uint32_t SOP_est(AccumulatorData *batt);

#endif /* INC_SOP_H_ */