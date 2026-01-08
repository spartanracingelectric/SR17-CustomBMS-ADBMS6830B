#include "main.h"
#include <stdio.h>
#include "accumulator.h"

void Accumulator_init(AccumulatorData *acc)
{
	acc->cell_volt_lowest = 0x0000;
	acc->cell_volt_highest = 0x0000;
	acc->cell_difference = 0x0000;
	acc->cell_temp_lowest = 0x0000;
	acc->cell_temp_highest = 0x0000;
	acc->sum_pack_voltage = 0x0000;
	acc->hvsens_pack_voltage = 0x0000;
	acc->balance_status[NUM_MOD] = 0x0000; 
	acc->soc = 0x00000000; // microamps!!!!!
	acc->current = 0x00000000;
}

void Accumulator_getMinVoltage(AccumulatorData *acc, ModuleData *mod)
{
	acc->cell_volt_lowest = mod[0].cell_volt[0];

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		for(int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (mod[moduleIndex].cell_volt[cellIndex] < acc->cell_volt_lowest) 
			{
				acc->cell_volt_lowest = mod[moduleIndex].cell_volt[cellIndex];
			}
		}
	}
}

void Accumulator_getMaxVoltage(AccumulatorData *acc, ModuleData *mod){
	acc->cell_volt_highest = mod[0].cell_volt[0];

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		for(int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (mod[moduleIndex].cell_volt[cellIndex] > acc->cell_volt_highest) 
			{
				acc->cell_volt_highest = mod[moduleIndex].cell_volt[cellIndex];
			}
		}
	}
}


/* ===== Pack Voltage Summation ===============================================
 * Accumulator_getTotalVoltage():
 *  - Sums per-cell voltages from ModuleData (assumed millivolts).
 *  - Converts to centivolts (÷100) and stores in batt->sum_pack_voltage.
 */
void Accumulator_getTotalVoltage(AccumulatorData *batt, ModuleData *mod)
{
	uint32_t totalVoltage = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++) 
		{
			totalVoltage += mod[moduleIndex].cell_volt[cellIndex]; //get sum in millivolts
		}
	}
	
	batt->sum_pack_voltage = (uint16_t)(totalVoltage / 100);  //mV → cV
}



