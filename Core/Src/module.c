/**
 * @file    module.c
 */
#include "module.h"
#include "main.h"
#include "usart.h"
#include <adbms6830b.h>
#include <math.h>
#include <stdio.h>

void Module_init(ModuleData *mod)
{
	for (int modIndex = 0; modIndex < NUM_MOD; modIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			mod[modIndex].cellVoltage_mV[cellIndex] = INT16_MAX;
		}
		for (int thermIndex = 0; thermIndex < NUM_THERM_PER_MOD; thermIndex++)
		{
			mod[modIndex].pointTemp_C[thermIndex] = 0xFF;
		}
		mod[modIndex].averageCellVoltage_mV = INT16_MAX;
		mod[modIndex].average_temp = 0xFFFF;
		mod[modIndex].totalCellVoltage_mV = INT32_MAX;
	}
}

void Module_getAverageCellVoltages(ModuleData *mod)
{
	// Only need to start conversions once in continuous mode
	static bool hasStartedAverageConversions = false;
	if (!hasStartedAverageConversions)
	{
		ADBMS_startCellVoltageConversions(REDUNDANT_MODE_OFF, CONTINUOUS_MODE_ON, DISCHARGE_MODE_OFF, FILTER_RESET_MODE_ON, OPEN_WIRE_MODE_ALL_OFF);
		hasStartedAverageConversions = true;
	}
	ADBMS_getAverageCellVoltages(mod);
}

void Module_getCellVoltages(ModuleData module[])
{
	ADBMS_startCellVoltageConversions(REDUNDANT_MODE_OFF, CONTINUOUS_MODE_OFF, DISCHARGE_MODE_OFF, FILTER_RESET_MODE_ON, OPEN_WIRE_MODE_ALL_OFF);
	ADBMS_getCellVoltages(module);
}

void Module_convertGpioVoltageToTemp(ModuleData *modData)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int tempIndex = 0; tempIndex < NUM_THERM_PER_MOD; tempIndex++)
		{
			float referenceVoltage = modData[moduleIndex].vref2;
			float outputVoltage = modData[moduleIndex].gpioVoltage_mV[tempIndex];

			// Guard against invalid values
			if (outputVoltage <= 0 || outputVoltage >= (referenceVoltage - VREF2_MARGIN_MV))
			{
				// TODO: Set invalid temp fault
				modData[moduleIndex].pointTemp_C[tempIndex] = DISCONNECTED_TEMP_C;
				continue;
			}

			// Compute thermistor resistance
			float ntcResistance =
				PULL_UP_RESISTANCE_OHMS * ((float)outputVoltage / (float)(referenceVoltage - outputVoltage));

			// Apply Steinhart-Hart equation based on Vishay NTC RT calculator
			// T = 1/(A1 + B1 * LN(RT/R25) + C1 * LN(RT/R25)^2 + D1 * LN(RT/R25)^3) -273.15
			float lnRR = logf(ntcResistance / NOMINAL_RESISTANCE_25C_OHMS);
			float inverseTemperature_K = STEINHART_HART_COEFFICIENT_A + (STEINHART_HART_COEFFICIENT_B * lnRR) + (STEINHART_HART_COEFFICIENT_C * lnRR * lnRR) + (STEINHART_HART_COEFFICIENT_D * lnRR * lnRR * lnRR);

			// Convert to Celsius
			float temperature_K = 1.0f / fmaxf(inverseTemperature_K, 1e-12f); // guard against division by 0
			float temperature_C = temperature_K - KELVIN_OFFSET;
			modData[moduleIndex].pointTemp_C[tempIndex] = temperature_C;
		}
	}
}

void Module_getTemperatures(ModuleData *mod)
{
	ADBMS_startAuxConversions(OW_OFF, PUP_OFF, AUX_CHANNEL_ALL);
	ADBMS_getVref2(mod);
	ADBMS_getGpioVoltages(mod);
	Module_convertGpioVoltageToTemp(mod);
}

void Module_getMaxCellVoltage(ModuleData *module)
{
	int16_t maxCellVoltage = INT16_MIN;
	uint8_t maxCellIndex = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] >= maxCellVoltage)
			{
				maxCellVoltage = module[moduleIndex].cellVoltage_mV[cellIndex];
				maxCellIndex = cellIndex;
			}
		}
		module[moduleIndex].maxCellVoltage_mV = maxCellVoltage;
		module[moduleIndex].maxCellIndex = maxCellIndex;
		// printf("module %d min cell voltage: %d cell: %d\n", moduleIndex + 1, module[moduleIndex].maxCellVoltage_mV, maxCellIndex + 1);
	}
}

void Module_getMinCellVoltage(ModuleData *module)
{
	int16_t minCellVoltage = INT16_MAX;
	uint8_t minCellIndex = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] <= minCellVoltage)
			{
				minCellVoltage = module[moduleIndex].cellVoltage_mV[cellIndex];
				minCellIndex = cellIndex;
			}
		}
		module[moduleIndex].minCellVoltage_mV = minCellVoltage;
		module[moduleIndex].minCellIndex = minCellIndex;
		// printf("module %d min cell voltage: %d cell: %d\n", moduleIndex + 1, module[moduleIndex].minCellVoltage_mV, minCellIndex + 1);
	}
}

void Module_getTotalCellVoltage(ModuleData *module)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		int32_t totalCellVoltage_mV = 0;
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			totalCellVoltage_mV += module[moduleIndex].cellVoltage_mV[cellIndex];
		}
		module[moduleIndex].totalCellVoltage_mV = totalCellVoltage_mV;
		// printf("Module %d total cell voltage: %d\n", moduleIndex + 1, module[moduleIndex].totalCellVoltage_mV);
	}
}

void Module_getAverageCellVoltage(ModuleData *module)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		module[moduleIndex].averageCellVoltage_mV = module[moduleIndex].totalCellVoltage_mV / NUM_CELL_PER_MOD;
		// printf("Module %d avg cell voltage: %d\n", moduleIndex + 1, module[moduleIndex].averageCellVoltage_mV);
	}
}

void Module_getStats(ModuleData *module)
{
	Module_getMaxCellVoltage(module);
	Module_getMinCellVoltage(module);
	Module_getTotalCellVoltage(module);
	Module_getAverageCellVoltage(module);
}
