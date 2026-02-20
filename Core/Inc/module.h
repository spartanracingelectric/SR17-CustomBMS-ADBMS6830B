#ifndef INC_MODULE_H_
#define INC_MODULE_H_

#include <stdbool.h>
#include <stdint.h>

#define NUM_MODULES_TOTAL 1
#define NUM_CELLS_PER_MODULE 14
#define NUM_CELLS_TOTAL (NUM_MODULES_TOTAL * NUM_CELLS_PER_MODULE)
#define NUM_THERMISTORS_PER_MODULE 10
#define NUM_THERMISTORS_TOTAL (NUM_MODULES_TOTAL * NUM_THERMISTORS_PER_MODULE)

#define MAX_CELL_CAPACITY 3000

#define STEINHART_HART_COEFFICIENT_A 3.3540164e-03f
#define STEINHART_HART_COEFFICIENT_B 3.0740376e-04f
#define STEINHART_HART_COEFFICIENT_C 1.0191529e-06f
#define STEINHART_HART_COEFFICIENT_D 9.0937118e-08f
#define NOMINAL_RESISTANCE_25C_OHMS 10000.0f
#define PULL_UP_RESISTANCE_OHMS 10000.0f
#define KELVIN_OFFSET 273.15f
#define DISCONNECTED_TEMP_C 0.0f
#define VREF2_MARGIN_MV 10.0f

typedef struct ModuleData
{
	int16_t cellVoltage_mV[NUM_CELLS_PER_MODULE];
	uint16_t redundantCellVoltage_mV[NUM_CELLS_PER_MODULE];
	int16_t gpio_volt[NUM_THERMISTORS_PER_MODULE];
	int16_t pointTemp_C[NUM_THERMISTORS_PER_MODULE];
	int16_t vref2;
	int16_t averageCellVoltage_mV;
	int16_t averageTemp_C;
	int32_t totalCellVoltage_mV;
	int16_t pressure;
	int16_t humidity;
	uint16_t dew_point;
	int16_t maxCellVoltage_mV;
	int16_t minCellVoltage_mV;
	uint8_t sid[6];
	bool pecError;
	uint8_t maxCellIndex;
	uint8_t minCellIndex;
} ModuleData;

void Module_init(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getCellVoltages(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getMaxCellVoltage(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getMinCellVoltage(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getTotalCellVoltage(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getAverageCellVoltage(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getStats(ModuleData module[NUM_MODULES_TOTAL]);
void Convert_GPIOVoltageToTemp(ModuleData module[NUM_MODULES_TOTAL]);
void Module_convertGpioVoltageToTemp(ModuleData module[NUM_MODULES_TOTAL]);
void Module_getTemperatures(ModuleData module[NUM_MODULES_TOTAL]);

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
