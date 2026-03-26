/**
 * @file    module.h
 * @brief   Public interface for module-level sensing and conversions (cells & env).
 */

#ifndef INC_MODULE_H_
#define INC_MODULE_H_

#include "main.h"

typedef struct ModuleData
{
	int16_t cellVoltage_mV[NUM_CELL_PER_MOD];
	uint16_t redundantCellVoltage_mV[NUM_CELL_PER_MOD];
	int16_t gpioVoltage_mV[NUM_THERM_PER_MOD];
	uint16_t pointTemp_C[NUM_THERM_PER_MOD];
	int16_t vref2;
	int16_t averageCellVoltage_mV;
	uint16_t averagePointTemperature_C;
	uint16_t maxPointTemperature_C;
	uint16_t minPointTemperature_C;
	uint8_t maxPointTemperatureIndex;
	uint8_t minPointTemperatureIndex;
	int32_t totalCellVoltage_mV;
	int16_t maxCellVoltage_mV;
	int16_t minCellVoltage_mV;
	uint8_t sid[6];
	bool pecError;
	uint8_t maxCellIndex;
	uint8_t minCellIndex;
} ModuleData;

#define STEINHART_HART_COEFFICIENT_A 3.3540164e-03f
#define STEINHART_HART_COEFFICIENT_B 3.0740376e-04f
#define STEINHART_HART_COEFFICIENT_C 1.0191529e-06f
#define STEINHART_HART_COEFFICIENT_D 9.0937118e-08f
#define NOMINAL_RESISTANCE_25C_OHMS 10000.0f
#define PULL_UP_RESISTANCE_OHMS 10000.0f
#define KELVIN_OFFSET 273.15f
#define DISCONNECTED_TEMP_C 0.0f
#define VREF2_MARGIN_MV 10.0f

void Module_init(ModuleData *mod);

void Module_getAverageCellVoltages(ModuleData *mod);
void Module_getCellVoltages(ModuleData module[]);
void Module_getMaxCellVoltage(ModuleData *module);
void Module_getMinCellVoltage(ModuleData *module);
void Module_getTotalCellVoltage(ModuleData *module);
void Module_getAverageCellVoltage(ModuleData *module);
void Module_getVoltageStats(ModuleData *mod);

void Module_convertGpioVoltageToTemp(ModuleData *modData);
void Module_getTemperatures(ModuleData *mod);
void Module_getAveragePointTemperature(ModuleData *mod);
void Module_getMaxPointTemperature(ModuleData *mod);
void Module_getMinPointTemperature(ModuleData *mod);
void Module_getTemperatureStats(ModuleData *mod);

#endif /* INC_MODULE_H_ */
