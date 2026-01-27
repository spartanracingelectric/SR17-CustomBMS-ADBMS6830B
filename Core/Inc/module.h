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
#define TEMPE_INVERCED_T0 (1.0f / 298.15f)
#define TEMP_Rp 10000.0f
#define TEMP_R0 10000.0f
#define TEMP_INVERCED_BETA (1.0f / 3435.0f)
#define TEMP_KELVIN 273.15f
#define LTC6811_Vdd 51450.0f

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
/* ===== Public API: Cell Voltages ============================================
 * Read_Volt():
 *  - Triggers and collects cell voltage measurements for all modules.
 *  - Populates ModuleData cell_volt[] (and may update per-module aggregates).
 */
void Module_getVoltages(ModuleData *mod);

/* ===== Public API: Temperature Conversions ==================================
 * Get_Actual_Temps():
 *  - Convert a raw temperature-related reading 'data' for device 'dev_idx'
 *    and logical index 'tempindex' into an entry of 'actual_temp' array.
 *
 */
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
void Atmos_Temp_To_Celsius(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);
void ADC_To_Humidity(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data);


/* ===== Public API: Derived Metrics ==========================================
 * Get_Dew_Point():
 *  - Compute dew point from ambient temperature and relative humidity stored
 *    in ModuleData and write the result back into ModuleData->dew_point.
 */
void Get_Dew_Point(ModuleData *mod);

#endif /* INC_MODULE_H_ */
