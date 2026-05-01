/**
 * @file    soc.h
 */
#ifndef INC_SOC_H_
#define INC_SOC_H_

#include "main.h"
#include "accumulator.h"
#include "module.h"

#define NUM_OCV_TABLES 3

#define MAX_CELL_CAPACITY_MAH 5000
#define PACK_CAPACITY_UAH (MAX_CELL_CAPACITY_MAH * NUM_CELLS_IN_PARALLEL * 1000)

#define SOC_SAVE_THRESHOLD_UAH (PACK_CAPACITY_UAH / 100)

#define SOC_MIN_CURRENT_MA  1000 // 1A, below this treat as noise
#define SOC_MIN_HV_PACK_VOLTAGE_CV 1000 

void SOC_updateCharge(AccumulatorData *batt, uint32_t elapsed_time);
void SOC_init(AccumulatorData *batt, ModuleData *mod);
float SOC_getPercent(AccumulatorData *batt);

#endif
