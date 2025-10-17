/**
 * @file    module.h
 * @brief   Public interface for module-level sensing and conversions (cells & env).
 *
 * This header collects:
 *  - ADC mode/timing macros for ADBMS/LTC measurement control
 *  - Sensor limits and electrical constants for conversions
 *  - Public APIs to read cell voltages, temperatures, and ambient sensors
 *  - Helpers to convert raw ADC/AUX data to engineering units (°C, %RH, kPa) and
 *    to derive secondary metrics (dew point)
 *
 * Conventions:
 *  - Voltage units are millivolts (mV) unless noted.
 *  - Temperature conversions return integer-scaled values (project-defined);
 *    see the .c implementation for exact scaling placed into ModuleData.
 *  - AUX/analog channels are read via the ADBMS/LTC AUX path; indices are
 *    provided by the caller (dev_idx, tempindex) to select device/channel.
 *  - Array sizes derive from NUM_MOD / NUM_CELL_PER_MOD / NUM_THERM_TOTAL in main.h.
 */
#ifndef INC_MODULE_H_
#define INC_MODULE_H_

#include "main.h"

#include <stdint.h>

/* ===== ADC Modes & Timing (ADBMS/LTC) =======================================
 * MD_*: conversion speed/bandwidth trade-offs for voltage/aux conversions.
 *  - MD_FAST     ≈ 27 kHz (lowest latency, highest noise)
 *  - MD_NORMAL   ≈ 7 kHz  (balanced)
 *  - MD_FILTERED ≈ 26 Hz  (heavily filtered, slow)
 *
 * *_DELAY: suggested software wait/poll intervals (ms) that match the mode.
 * CELL_CH_ALL: select all series-cell channels.
 * DCP_DISABLED: disable discharge during conversion (balancing paused).
 */
#define MD_FAST 1		//27kHz
#define MD_NORMAL 2		//7kHz
#define MD_FILTERED 3	//26Hz
#define FAST_DELAY 2
#define NORMAL_DELAY 4
#define FILTERD_DELAY 202
#define CELL_CH_ALL 0
#define DCP_DISABLED 0

/* ===== Sensor/Conversion Limits & References ================================
 * SENSOR_MIN_TEMP / SENSOR_MAX_TEMP: validity clamp for temperature calc.
 * LTC6811_Vdd: reference supply for scaling certain AUX channels (mV).
 */
#define SENSOR_MIN_TEMP -40.0f
#define SENSOR_MAX_TEMP 125.0f

#define LTC6811_Vdd 51450.0f

void Module_init(ModuleData *mod);
/* ===== Public API: Cell Voltages ============================================
 * Read_Volt():
 *  - Triggers and collects cell voltage measurements for all modules.
 *  - Populates ModuleData cell_volt[] (and may update per-module aggregates).
 */
void Read_Volt(ModuleData *mod);

/* ===== Public API: Temperature Conversions ==================================
 * Get_Actual_Temps():
 *  - Convert a raw temperature-related reading 'data' for device 'dev_idx'
 *    and logical index 'tempindex' into an entry of 'actual_temp' array.
 *
 * Read_Temp():
 *  - Read temperature-related AUX channels selected by 'tempindex' and
 *    place raw values into read_temp[] and read_auxreg[] for later conversion.
 */
void Get_Actual_Temps(uint8_t dev_idx, uint8_t tempindex, uint16_t *actual_temp, uint16_t data);
void Read_Temp(uint8_t tempindex, uint16_t *read_temp, uint16_t *read_auxreg);

/* ===== Public API: Ambient Sensors (Pressure / Air Temp / Humidity) =========
 * Convert_Analog_To_Pressure():
 *  - Convert raw ADC/AUX code 'adc_data' for module 'dev_idx' into pressure
 *    and store into ModuleData (units per project convention).
 *
 * Atmos_Temp_To_Celsius():
 *  - Convert raw ADC/AUX code to ambient temperature for module 'dev_idx'
 *    and store into ModuleData (typically °C scaled to integer).
 *
 * ADC_To_Humidity():
 *  - Convert raw ADC/AUX code to relative humidity for module 'dev_idx'
 *    and store into ModuleData (typically %RH scaled to integer).
 *
 * Read_Pressure()/Read_Atmos_Temp()/Read_Humidity():
 *  - Acquire the respective channels for all modules and update ModuleData.
 */
void Convert_Analog_To_Pressure(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);
void Atmos_Temp_To_Celsius(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);
void ADC_To_Humidity(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);
void Read_Pressure(ModuleData *mod);
void Read_Atmos_Temp(ModuleData *mod);
void Read_Humidity(ModuleData *mod);

/* ===== Public API: Derived Metrics ==========================================
 * Get_Dew_Point():
 *  - Compute dew point from ambient temperature and relative humidity stored
 *    in ModuleData and write the result back into ModuleData->dew_point.
 */
void Get_Dew_Point(ModuleData *mod);

#endif /* INC_MODULE_H_ */
