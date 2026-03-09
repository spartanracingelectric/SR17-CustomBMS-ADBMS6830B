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
	acc->averagePointTemp_C = 0x0000;
	acc->sumPackVoltage_cV = 0x0000;
	acc->hvsens_pack_voltage = 0x0000;
	acc->soc = 0x00000000; // microamps!!!!!
	acc->current = 0x00000000;
}

void Accumulator_getMinVoltage(AccumulatorData *acc, ModuleData *mod)
{
	// TODO: Only look at module min cell voltage instead of every voltage in module
	acc->minCellVoltage_mV = mod[0].cellVoltage_mV[0];

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (mod[moduleIndex].cellVoltage_mV[cellIndex] < acc->minCellVoltage_mV)
			{
				acc->minCellVoltage_mV = mod[moduleIndex].cellVoltage_mV[cellIndex];
			}
		}
	}
}

void Accumulator_getMaxVoltage(AccumulatorData *acc, ModuleData *mod)
{
	// TODO: Only look at module max cell voltage instead of every voltage in module
	acc->maxCellVoltage_mV = mod[0].cellVoltage_mV[0];

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (mod[moduleIndex].cellVoltage_mV[cellIndex] > acc->maxCellVoltage_mV)
			{
				acc->maxCellVoltage_mV = mod[moduleIndex].cellVoltage_mV[cellIndex];
			}
		}
	}
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

void Accumulator_getTemperatureStats(AccumulatorData *acc, ModuleData *mod){
	int32_t avgTempSum = mod[0].averagePointTemperature_C;
	uint16_t maxPointTemp = mod[0].maxPointTemperature_C;
	uint16_t minPointTemp = mod[0].minPointTemperature_C;

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
