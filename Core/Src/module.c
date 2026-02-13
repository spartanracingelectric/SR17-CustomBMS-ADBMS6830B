/**
 * @file    module.c
 * @brief   Module-level sensing and conversions (cell voltages, NTC temps, ambient sensors).
 *
 * This file provides:
 *  - Read_Volt(): trigger ADCV and collect averaged per-cell voltages into ModuleData.
 *  - Get_Actual_Temps(): NTC linearization via Beta model → Celsius.
 *  - ADC_To_Pressure(), Atmos_Temp_To_Celsius(), ADC_To_Humidity(): ambient sensor linearizations.
 *  - (commented examples) AUX/MUX read path for temps/pressure/air temp/humidity and dew point calc.
 *
 * Conventions:
 *  - Cell voltages are stored per-module in millivolts (mV) unless noted by the target field.
 *  - Ambient quantities are converted with simple linear models unless otherwise documented.
 *  - Temperatures are computed in °C using an NTC Beta approximation (not full 3-coef Steinhart–Hart).
 *  - Array indexing: module ‘dev_idx’ selects ModuleData[dev_idx]; thermistor index is ‘tempindex’.
 *
 * Assumptions:
 *  - NUM_MOD / NUM_CELL_PER_MOD / NUM_THERM_PER_MOD are defined in main.h.
 *  - LTC6811_Vdd defines the AUX reference for ratio-metric ambient sensors (in the same units as adc_data scale).
 *  - ADBMS_getAVGCellVoltages() handles SNAP/UNSNAP/PEC and writes into ModuleData.
 */
#include "module.h"
#include "main.h"
#include "usart.h"
#include <adbms6830b.h>
#include <math.h>
#include <stdio.h>

/* ===== NTC Thermistor Model (Beta Approximation) =============================
 * Parameters:
 *  - ntcNominal: resistance at ntcNominalTemp (typically 10kΩ @ 25°C).
 *  - ntcSeriesResistance: series/bias resistor value used in divider.
 *  - ntcBetaFactor: Beta constant (K); a single-slope approximation.
 *  - ntcNominalTemp: nominal temperature (°C) for ntcNominal.
 *
 * Computation (Get_Actual_Temps):
 *  1) Convert raw ‘data’ to NTC resistance using the divider equation.
 *  2) Apply Beta model to obtain Kelvin, then convert to °C.
 */
#define ntcNominal 10000.0f
#define ntcSeriesResistance 10000.0f
#define ntcBetaFactor 3435.0f
#define ntcNominalTemp 25.0f

static const float invNominalTemp = 1.0f / (ntcNominalTemp + 273.15f);
static const float invBetaFactor = 1.0f / ntcBetaFactor;

void Module_init(ModuleData *mod)
{
	for (int modIndex = 0; modIndex < NUM_MOD; modIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			mod[modIndex].cell_volt[cellIndex] = 0xFFFF;
		}
		for (int thermIndex = 0; thermIndex < NUM_THERM_PER_MOD; thermIndex++)
		{
			mod[modIndex].pointTemp_C[thermIndex] = 0xFF;
		}
		mod[modIndex].averageCellVoltage_mV = INT16_MAX;
		mod[modIndex].average_temp = 0xFFFF;
		mod[modIndex].totalCellVoltage_mV = INT32_MAX;
		mod[modIndex].pressure = 0xFFFF;
		mod[modIndex].humidity = 0xFFFF;
		//		mod[modIndex].atmos_temp = 0xFFFF;
	}
}

void Module_getVoltages(ModuleData *mod)
{
	ADBMS_getCellVoltages(mod);
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
void ADC_To_Pressure(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data)
{
	float psi = (float)adc_data / 325.0; // convert the adc value based on Vref

	mod[dev_idx].pressure = (uint16_t)(psi * 100); // relative to atmospheric pressure
}

void Atmos_Temp_To_Celsius(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data)
{
	float voltage_ratio = (float)adc_data / LTC6811_Vdd; // convert the adc value based on Vref

	float temperature_value = -66.875 + 218.75 * voltage_ratio; // Calculate pressure

	//    mod[dev_idx].atmos_temp = (uint16_t)temperature_value;
}

void ADC_To_Humidity(uint8_t dev_idx, ModuleData *mod, uint16_t adcValue)
{
	float voltage_ratio = (float)adcValue / LTC6811_Vdd;

	float humidity_value = (-12.5 + 125.0 * voltage_ratio); // Calculate pressure

	mod[dev_idx].humidity = (uint16_t)(humidity_value);
}

/* ===== NTC Temperature Conversion ===========================================
 * Get_Actual_Temps():
 *  - Converts a raw AUX reading ‘data’ into an NTC resistance using a divider
 *    that effectively computes:
 *        R_ntc = (ntcSeriesResistance) / ( (Vref/Vmeas) - 1 )
 *    (The ‘30000.0f/data - 1’ constant implements a project-specific scale.)
 *  - Applies Beta model to compute Kelvin, then converts to °C.
 *  - Writes the result into ‘actual_temp[dev_idx * NUM_THERM_PER_MOD + tempindex]’.
 *  - If data==0, writes 999.0f sentinel value (error) into the array.
 */
void Convert_GPIOVoltageToTemp(ModuleData *mod)
{
	for (int modIndex = 0; modIndex < NUM_MOD; modIndex++)
	{
		for (int tempIndex = 0; tempIndex < NUM_THERM_PER_MOD; tempIndex++)
		{
			printf("gpio volt %d", mod[modIndex].gpio_volt[tempIndex]);
			printf("vref2: %d", mod[modIndex].vref2);
			float TEMP_R = TEMP_Rp * (mod[modIndex].gpio_volt[tempIndex] /
									  (mod[modIndex].vref2 - mod[modIndex].gpio_volt[tempIndex]));
			float lnR_R0 = logf(TEMP_R / TEMP_R0);
			float TEMP_T = (1 / (TEMPE_INVERCED_T0 + TEMP_INVERCED_BETA * lnR_R0)) - TEMP_KELVIN;
			mod[modIndex].pointTemp_C[tempIndex] = TEMP_T;
			printf("Module %d, Cell %d old_Temp_C: %f\n", modIndex, tempIndex + 1, TEMP_T);
		}
	}
}

void Module_convertGpioVoltageToTemp(ModuleData *modData)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (int tempIndex = 0; tempIndex < NUM_THERM_PER_MOD; tempIndex++)
		{
			float referenceVoltage = modData[moduleIndex].vref2;
			float outputVoltage = modData[moduleIndex].gpio_volt[tempIndex];

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
			float inverseTemperature_K = STEINHART_HART_COEFFICIENT_A + (STEINHART_HART_COEFFICIENT_B * lnRR) +
										 (STEINHART_HART_COEFFICIENT_C * lnRR * lnRR) +
										 (STEINHART_HART_COEFFICIENT_D * lnRR * lnRR * lnRR);

			// Convert to Celsius
			float temperature_K = 1.0f / fmaxf(inverseTemperature_K, 1e-12f); // guard against division by 0
			float temperature_C = temperature_K - KELVIN_OFFSET;
			modData[moduleIndex].pointTemp_C[tempIndex] = temperature_C;
		}
	}
}

void Get_Dew_Point(ModuleData *mod)
{
	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++)
	{
		uint16_t humidity = mod[dev_idx].humidity;
		//		uint16_t atmos_temp = mod[dev_idx].atmos_temp;

		// simple approximation that is accurate to within 1 deg C
		// Only works well when Relative Humidity is above 50%

		//		mod[dev_idx].dew_point = atmos_temp - ((float)(100 - humidity) / 5);
	}
}

/* ===== (Examples/Disabled) AUX MUX Read Path ================================
 * The following routines illustrate how to steer an I²C MUX via COMM, kick an
 * AUX conversion, read back AUX groups, and post-process into engineering units.
 * They are left commented as TODO; enable and adapt to your hardware mapping.
 */
// TODO: gpio read
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
			if (module[moduleIndex].cell_volt[cellIndex] >= maxCellVoltage)
			{
				maxCellVoltage = module[moduleIndex].cell_volt[cellIndex];
				maxCellIndex = cellIndex;
			}
		}
		module[moduleIndex].maxCellVoltage_mV = maxCellVoltage;
		module[moduleIndex].maxCellIndex = maxCellIndex;
		printf("module %d min cell voltage: %d cell: %d\n", moduleIndex + 1, module[moduleIndex].maxCellVoltage_mV, maxCellIndex + 1);
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
			if (module[moduleIndex].cell_volt[cellIndex] <= minCellVoltage)
			{
				minCellVoltage = module[moduleIndex].cell_volt[cellIndex];
				minCellIndex = cellIndex;
			}
		}
		module[moduleIndex].minCellVoltage_mV = minCellVoltage;
		module[moduleIndex].minCellIndex = minCellIndex;
		printf("module %d min cell voltage: %d cell: %d\n", moduleIndex + 1, module[moduleIndex].minCellVoltage_mV, minCellIndex + 1);
	}
}

void Module_getTotalCellVoltage(ModuleData *module)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		int32_t totalCellVoltage_mV= 0;
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			totalCellVoltage_mV += module[moduleIndex].cell_volt[cellIndex];
		}
		module[moduleIndex].totalCellVoltage_mV = totalCellVoltage_mV;
		printf("Module %d total cell voltage: %d\n", moduleIndex + 1, module[moduleIndex].totalCellVoltage_mV);
	}
}

void Module_getAverageCellVoltage(ModuleData *module)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		module[moduleIndex].averageCellVoltage_mV = module[moduleIndex].totalCellVoltage_mV / NUM_CELL_PER_MOD;
		printf("Module %d avg cell voltage: %d\n", moduleIndex + 1, module[moduleIndex].averageCellVoltage_mV);
	}
}

void Module_getStats(ModuleData *module)
{
	Module_getMaxCellVoltage(module);
	Module_getMinCellVoltage(module);
	Module_getTotalCellVoltage(module);
	Module_getAverageCellVoltage(module);
}
