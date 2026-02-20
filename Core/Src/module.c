/**
 * @file    module.c
 * @brief   Module-level sensing and conversions (cell voltages, NTC temps, ambient sensors).
 */

#include "module.h"
#include <adbms6830b.h>
#include <math.h>
#include <stdint.h>

void Module_init(ModuleData module[NUM_MODULES_TOTAL])
{
	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			module[moduleIndex].cellVoltage_mV[cellIndex] = INT16_MAX;
		}
		for (int tempIndex = 0; tempIndex < NUM_THERMISTORS_PER_MODULE; tempIndex++)
		{
			module[moduleIndex].pointTemp_C[tempIndex] = INT16_MAX;
		}
		module[moduleIndex].averageCellVoltage_mV = INT16_MAX;
		module[moduleIndex].averageTemp_C = INT16_MAX;
		module[moduleIndex].totalCellVoltage_mV = INT32_MAX;
		module[moduleIndex].pressure = INT16_MAX;
		module[moduleIndex].humidity = INT16_MAX;
	}
}

void Module_getCellVoltages(ModuleData module[NUM_MODULES_TOTAL])
{
	ADBMS_getCellVoltages(module);
}

/* ===== Ambient Sensors: Linearizations ======================================
 * ADC_To_Pressure():
 *  - Linearly maps raw ‘adc_data’ to PSI using a project-specific scale (adc/325.0).
 *  - Stores centi-PSI (×100) as uint16_t in ModuleData.pressure.
 *
 * Atmos_Temp_To_Celsius():
 *  - Ratio-metric conversion against LTC6811_Vdd, then linear map:
 *      T(°C) = -66.875 + 218.75 * (adc/LTC6811_Vdd)
 *
 * ADC_To_Humidity():
 *  - Ratio-metric conversion, then linear map:
 *      RH(%) = -12.5 + 125.0 * (adc/LTC6811_Vdd)
 */

// void ADC_To_Pressure(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data)
// {
// 	float psi = (float)adc_data / 325.0; // convert the adc value based on Vref
//
// 	mod[dev_idx].pressure = (uint16_t)(psi * 100); // relative to atmospheric pressure
// }
//
// void Atmos_Temp_To_Celsius(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data)
// {
// 	float voltage_ratio = (float)adc_data / LTC6811_Vdd; // convert the adc value based on Vref
//
// 	float temperature_value = -66.875 + 218.75 * voltage_ratio; // Calculate pressure
//
// 	//    mod[dev_idx].atmos_temp = (uint16_t)temperature_value;
// }
//
// void ADC_To_Humidity(uint8_t dev_idx, ModuleData *mod, uint16_t adcValue)
// {
// 	float voltage_ratio = (float)adcValue / LTC6811_Vdd;
//
// 	float humidity_value = (-12.5 + 125.0 * voltage_ratio); // Calculate pressure
//
// 	mod[dev_idx].humidity = (uint16_t)(humidity_value);
// }

void Module_convertGpioVoltageToTemp(ModuleData module[NUM_MODULES_TOTAL])
{
	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int tempIndex = 0; tempIndex < NUM_THERMISTORS_PER_MODULE; tempIndex++)
		{
			float referenceVoltage = module[moduleIndex].vref2;
			float outputVoltage = module[moduleIndex].gpio_volt[tempIndex];

			// Guard against invalid values
			if (outputVoltage <= 0 || outputVoltage >= (referenceVoltage - VREF2_MARGIN_MV))
			{
				module[moduleIndex].pointTemp_C[tempIndex] = DISCONNECTED_TEMP_C;
				continue;
			}

			// Compute thermistor resistance
			float ntcResistance = PULL_UP_RESISTANCE_OHMS * ((float)outputVoltage / (float)(referenceVoltage - outputVoltage));

			// Apply Steinhart-Hart equation based on Vishay NTC RT calculator
			// T = 1/(A1 + B1 * LN(RT/R25) + C1 * LN(RT/R25)^2 + D1 * LN(RT/R25)^3) -273.15
			float lnRR = logf(ntcResistance / NOMINAL_RESISTANCE_25C_OHMS);
			float inverseTemperature_K = STEINHART_HART_COEFFICIENT_A + (STEINHART_HART_COEFFICIENT_B * lnRR) + (STEINHART_HART_COEFFICIENT_C * lnRR * lnRR) + (STEINHART_HART_COEFFICIENT_D * lnRR * lnRR * lnRR);

			// Convert to Celsius
			float temperature_K = 1.0f / fmaxf(inverseTemperature_K, 1e-12f); // guard against division by 0
			float temperature_C = temperature_K - KELVIN_OFFSET;
			module[moduleIndex].pointTemp_C[tempIndex] = temperature_C;
		}
	}
}

// void Get_Dew_Point(ModuleData *mod)
// {
// 	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++)
// 	{
// 		uint16_t humidity = mod[dev_idx].humidity;
// 		//		uint16_t atmos_temp = mod[dev_idx].atmos_temp;
//
// 		// simple approximation that is accurate to within 1 deg C
// 		// Only works well when Relative Humidity is above 50%
//
// 		//		mod[dev_idx].dew_point = atmos_temp - ((float)(100 - humidity) / 5);
// 	}
// }

void Module_getTemperatures(ModuleData module[NUM_MODULES_TOTAL])
{
	ADBMS_startAuxConversions(OW_OFF, PUP_OFF, AUX_CHANNEL_ALL);
	ADBMS_getVref2(module);
	ADBMS_getGpioVoltages(module);
	Module_convertGpioVoltageToTemp(module);
}

void Module_getMaxCellVoltage(ModuleData module[NUM_MODULES_TOTAL])
{
	int16_t maxCellVoltage = INT16_MIN;
	uint8_t maxCellIndex = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] >= maxCellVoltage)
			{
				maxCellVoltage = module[moduleIndex].cellVoltage_mV[cellIndex];
				maxCellIndex = cellIndex;
			}
		}
		module[moduleIndex].maxCellVoltage_mV = maxCellVoltage;
		module[moduleIndex].maxCellIndex = maxCellIndex;
	}
}

void Module_getMinCellVoltage(ModuleData module[NUM_MODULES_TOTAL])
{
	int16_t minCellVoltage = INT16_MAX;
	uint8_t minCellIndex = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] <= minCellVoltage)
			{
				minCellVoltage = module[moduleIndex].cellVoltage_mV[cellIndex];
				minCellIndex = cellIndex;
			}
		}
		module[moduleIndex].minCellVoltage_mV = minCellVoltage;
		module[moduleIndex].minCellIndex = minCellIndex;
	}
}

void Module_getTotalCellVoltage(ModuleData module[NUM_MODULES_TOTAL])
{
	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		int32_t totalCellVoltage_mV = 0;
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			totalCellVoltage_mV += module[moduleIndex].cellVoltage_mV[cellIndex];
		}
		module[moduleIndex].totalCellVoltage_mV = totalCellVoltage_mV;
	}
}

void Module_getAverageCellVoltage(ModuleData module[NUM_MODULES_TOTAL])
{
	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		module[moduleIndex].averageCellVoltage_mV = module[moduleIndex].totalCellVoltage_mV / NUM_CELLS_PER_MODULE;
	}
}

void Module_getStats(ModuleData module[NUM_MODULES_TOTAL])
{
	Module_getMaxCellVoltage(module);
	Module_getMinCellVoltage(module);
	Module_getTotalCellVoltage(module);
	Module_getAverageCellVoltage(module);
}
