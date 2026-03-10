#include "accumulator.h"
#include "main.h"
#include <stdio.h>

void Accumulator_init(AccumulatorData *acc)
{
	acc->minCellVoltage_mV = 0x0000;
	acc->maxCellVoltage_mV = 0x0000;
	acc->cellImbalance_mV = 0x0000;
	acc->minCellTemp_C = 0x0000;
	acc->maxCellTemp_C = 0x0000;
	acc->sumPackVoltage_cV = 0x0000;
	acc->hvsens_pack_voltage = 0x0000;
	acc->soc = 0x00000000; // microamps!!!!!
	acc->current = 0x00000000;
}

void Accumulator_getMinVoltage(AccumulatorData *acc, ModuleData *mod)
{
	int16_t minVoltage = mod[0].minCellVoltage_mV;
	for(uint8_t moduleIndex = 1; moduleIndex < NUM_MOD; moduleIndex++){
		if(mod[moduleIndex].minCellVoltage_mV < minVoltage){
			minVoltage = mod[moduleIndex].minCellVoltage_mV;
		}
	}
	acc->minCellVoltage_mV = minVoltage;
}

void Accumulator_getMaxVoltage(AccumulatorData *acc, ModuleData *mod)
{
	int16_t maxVoltage = mod[0].maxCellVoltage_mV;
	for(uint8_t moduleIndex = 1; moduleIndex < NUM_MOD; moduleIndex++){
		if(mod[moduleIndex].maxCellVoltage_mV > maxVoltage){
			maxVoltage = mod[moduleIndex].maxCellVoltage_mV;
		}
	}
	acc->maxCellVoltage_mV = maxVoltage;
}

void Accumulator_getVoltageStats(AccumulatorData *acc, ModuleData *mod){
	int32_t averageVoltageSum = mod[0].averageCellVoltage_mV;
	int16_t maxVoltage = mod[0].maxCellVoltage_mV;
	int16_t minVoltage = mod[0].minCellVoltage_mV;

	for(uint8_t moduleIndex = 1; moduleIndex < NUM_MOD; moduleIndex++){
		ModuleData	*module = &mod[moduleIndex];
		averageVoltageSum += module->averageCellVoltage_mV;
		if(module->maxCellVoltage_mV > maxVoltage){
			maxVoltage = module->maxCellVoltage_mV;
		}
		if(module->minCellVoltage_mV < minVoltage){
			minVoltage = module->minCellVoltage_mV;
		}
	}
	acc->averageCellVoltage_mV = averageVoltageSum / NUM_MOD;
	acc->maxCellVoltage_mV = maxVoltage;
	acc->minCellVoltage_mV = minVoltage;
}

void Accumulator_getTotalVoltage(AccumulatorData *batt, ModuleData *mod)
{
	int32_t totalVoltage_mV = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			totalVoltage_mV += mod[moduleIndex].cellVoltage_mV[cellIndex];
		}
	}

	batt->sumPackVoltage_cV = (int16_t)(totalVoltage_mV / 100);
}
