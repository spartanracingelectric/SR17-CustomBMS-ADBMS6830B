/**
 * @file    module.h
 * @brief   Public interface for module-level sensing and conversions (cells & env).
 */

#ifndef INC_MODULE_H_
#define INC_MODULE_H_

#include "main.h"
#include <stdint.h>

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
void Module_getCellVoltages(ModuleData *mod);
void Module_getMaxCellVoltage(ModuleData *module);
void Module_getMinCellVoltage(ModuleData *module);
void Module_getTotalCellVoltage(ModuleData *module);
void Module_getAverageCellVoltage(ModuleData *module);
void Module_getStats(ModuleData *module);
void Convert_GPIOVoltageToTemp(ModuleData *mod);
void Module_convertGpioVoltageToTemp(ModuleData *modData);
void Module_getTemperatures(ModuleData *mod);

/* ===== Public API: Ambient Sensors (Pressure / Air Temp / Humidity) =========
 * Atmos_Temp_To_Celsius():
 *  - Convert raw ADC/AUX code to ambient temperature for module 'dev_idx'
 *    and store into ModuleData (typically °C scaled to integer).
 *
 * ADC_To_Humidity():
 *  - Convert raw ADC/AUX code to relative humidity for module 'dev_idx'
 *    and store into ModuleData (typically %RH scaled to integer).
 */
// void Atmos_Temp_To_Celsius(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);
// void ADC_To_Humidity(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);

/* ===== Public API: Derived Metrics ==========================================
 * Get_Dew_Point():
 *  - Compute dew point from ambient temperature and relative humidity stored
 *    in ModuleData and write the result back into ModuleData->dew_point.
 */
// void Get_Dew_Point(ModuleData *mod);

#endif /* INC_MODULE_H_ */
