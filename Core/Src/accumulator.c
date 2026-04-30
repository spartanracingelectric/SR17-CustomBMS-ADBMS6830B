#include "accumulator.h"
#include <stdio.h>

void Accumulator_init(AccumulatorData *acc)
{
	acc->minCellVoltage_mV = 0x0000;
	acc->maxCellVoltage_mV = 0x0000;
	acc->cellImbalance_mV = 0x0000;
	acc->minCellTemp_C = 0x0000;
	acc->maxCellTemp_C = 0x0000;
	acc->averagePointTemp_C = 0x0000;
	acc->sumPackVoltage_cV = 0x0000;
	acc->hvSensePackVoltage_cV = 0x0000;
	acc->soc = 0x00000000;
	acc->shuntCurrent_mA = 0x00000000;
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
	uint32_t totalPackVoltage_mV = mod[0].totalCellVoltage_mV;

	for(uint8_t moduleIndex = 1; moduleIndex < NUM_MOD; moduleIndex++){
		ModuleData	*module = &mod[moduleIndex];
		averageVoltageSum += module->averageCellVoltage_mV;
		totalPackVoltage_mV += module->totalCellVoltage_mV;
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
	acc->cellImbalance_mV = maxVoltage - minVoltage;
	acc->sumPackVoltage_cV = totalPackVoltage_mV / 10;
}

void Accumulator_getTotalVoltage(AccumulatorData *batt, ModuleData *mod)
{
	int32_t totalVoltage_mV = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		totalVoltage_mV += mod[moduleIndex].totalCellVoltage_mV;
	}

	batt->sumPackVoltage_cV = (int16_t)(totalVoltage_mV / 100);
}

void Accumulator_getTemperatureStats(AccumulatorData *acc, ModuleData *mod){
	int32_t avgTempSum = mod[0].averagePointTemperature_C;
	int16_t maxPointTemp = mod[0].maxPointTemperature_C;
	int16_t minPointTemp = mod[0].minPointTemperature_C;

	for(uint8_t moduleIndex = 1; moduleIndex < NUM_MOD; moduleIndex++){
		ModuleData* module = &mod[moduleIndex];
		avgTempSum += module->averagePointTemperature_C;
		if(module->maxPointTemperature_C > maxPointTemp){
			maxPointTemp = module->maxPointTemperature_C;
		}
		if(module->minPointTemperature_C < minPointTemp){
			minPointTemp = module->minPointTemperature_C;
		}
	}
	acc->averagePointTemp_C = avgTempSum / NUM_MOD;
	acc->maxCellTemp_C = maxPointTemp;
	acc->minCellTemp_C = minPointTemp;
}


