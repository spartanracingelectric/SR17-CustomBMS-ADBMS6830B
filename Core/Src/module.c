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
#include <adbms6830b.h>
#include "module.h"
#include <math.h>
#include <stdio.h>
#include "usart.h"

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

void Module_init(ModuleData *mod){
	for(int modIndex = 0; modIndex < NUM_MOD; modIndex++){
		for(int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++){
			mod[modIndex].cell_volt[cellIndex] = 0xFFFF;
		}
		for(int thermIndex = 0; thermIndex < NUM_THERM_TOTAL; thermIndex++){
			mod[modIndex].cell_temp[NUM_THERM_TOTAL] = 0xFF;
		}
		mod[modIndex].average_volt = 0xFFFF;
		mod[modIndex].average_temp = 0xFFFF;
		mod[modIndex].sum_volt_module = 0xFFFF;
		mod[modIndex].pressure = 0xFFFF;
		mod[modIndex].humidity = 0xFFFF;
		mod[modIndex].atmos_temp = 0xFFFF;
	}

}

// addresses provided by the mux (uses i2c protocol)
//static uint8_t BMS_MUX[][6] = {{ 0x69, 0x28, 0x0F, 0xF9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xE9, 0x7F, 0xF9 },
//								 { 0x69, 0x28, 0x0F, 0xD9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xC9, 0x7F, 0xF9 },
//								 { 0x69, 0x28, 0x0F, 0xB9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xA9, 0x7F, 0xF9 },
//								 { 0x69, 0x28, 0x0F, 0x99, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0x89, 0x7F, 0xF9 },
//								 { 0x69, 0x08, 0x0F, 0xF9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F, 0xE9, 0x7F, 0xF9 },
//								 { 0x69, 0x08, 0x0F, 0xD9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F, 0xC9, 0x7F, 0xF9 },
//							 	 { 0x69, 0x08, 0x0F, 0xB9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F, 0xA9, 0x7F, 0xF9 },
//								 { 0x69, 0x08, 0x0F, 0x99, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F, 0x89, 0x7F, 0xF9 } };

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
void ADC_To_Pressure(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data) {
    float psi = (float) adc_data / 325.0;  // convert the adc value based on Vref

    mod[dev_idx].pressure = (uint16_t)(psi * 100);  // relative to atmospheric pressure
}

void Atmos_Temp_To_Celsius(uint8_t dev_idx, ModuleData *mod, uint16_t adc_data) {
    float voltage_ratio = (float) adc_data / LTC6811_Vdd;  // convert the adc value based on Vref

    float temperature_value = -66.875 + 218.75 * voltage_ratio;  //Calculate pressure

    mod[dev_idx].atmos_temp = (uint16_t)temperature_value;
}

void ADC_To_Humidity(uint8_t dev_idx, ModuleData *mod, uint16_t adcValue) {
    float voltage_ratio = (float)adcValue / LTC6811_Vdd;

    float humidity_value = (-12.5 + 125.0 * voltage_ratio);  //Calculate pressure

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
void Get_Actual_Temps(uint8_t dev_idx, uint8_t tempindex, uint16_t *actual_temp, uint16_t data) {
    if (data == 0) {
        actual_temp[dev_idx * NUM_THERM_PER_MOD + tempindex] = 999.0f; // error value
        return;
    }

    float scalar = 30000.0f / (float)(data) - 1.0f;
    scalar = ntcSeriesResistance / scalar;

    float steinhart = scalar / ntcNominal;
    steinhart = log(steinhart);
    steinhart *= invBetaFactor;
    steinhart += invNominalTemp;
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;

    actual_temp[dev_idx * NUM_THERM_PER_MOD + tempindex] = steinhart;
}

/* ===== Cell Voltage Acquisition =============================================
 * Read_Volt():
 *  - Calls ADBMS_getAVGCellVoltages(mod) to read and average per-cell voltages
 *    across all devices. The callee is responsible for PEC checks and page reads.
 */
void Read_Volt(ModuleData *mod) {
//	printf("volt start\n");
	ADBMS_getAVGCellVoltages(mod);
//	printf("volt end\n");
}

/* ===== (Examples/Disabled) AUX MUX Read Path ================================
 * The following routines illustrate how to steer an I²C MUX via COMM, kick an
 * AUX conversion, read back AUX groups, and post-process into engineering units.
 * They are left commented as TODO; enable and adapt to your hardware mapping.
 */
//TODO: gpio read
//void Read_Temp(uint8_t tempindex, uint16_t *read_temp, uint16_t *read_auxreg) {
//
////	printf("Temperature read start\n");
//	LTC_SPI_writeCommunicationSetting(NUM_MOD, BMS_MUX[tempindex]);
//	LTC_SPI_requestData(2);
//	//end sending to mux to read temperatures
//	if (tempindex == 0) {
//		LTC_startADC_GPIO(MD_NORMAL, 1); //ADC mode: MD_FILTERED, MD_NORMAL, MD_FAST
//		HAL_Delay(NORMAL_DELAY + 2); //FAST_DELAY, NORMAL_DELAY, FILTERD_DELAY;
//	} else {
//		LTC_startADC_GPIO(MD_FAST, 1); //ADC mode: MD_FILTERED, MD_NORMAL, MD_FAST
//		HAL_Delay(FAST_DELAY); //FAST_DELAY, NORMAL_DELAY, FILTERD_DELAY;
//	}
//	if (!LTC_readGPIOs((uint16_t*) read_auxreg)) // Set to read back all aux registers
//			{
//		for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
//			//isoSPI_Idle_to_Ready();
//			// Assuming data format is [cell voltage, cell voltage, ..., PEC, PEC]
//			// PEC for each device is the last two bytes of its data segment
//			uint16_t data = read_auxreg[dev_idx * NUM_AUX_GROUP];
//			//read_temp[dev_idx * NUM_THERM_PER_MOD + tempindex] = data;
//			Get_Actual_Temps(dev_idx, tempindex, (uint16_t*) read_temp, data); //+5 because vref is the last reg
//		}
//	}
////	printf("Temperature read end\n");
//}
//
//
//void Read_Pressure(ModuleData *mod) {
//	LTC_SPI_writeCommunicationSetting(NUM_MOD, BMS_MUX[12]);
//	LTC_SPI_requestData(2);
//
//	LTC_startADC_GPIO(MD_NORMAL, 1); //ADC mode: MD_FILTERED, MD_NORMAL, MD_FAST
//    HAL_Delay(NORMAL_DELAY); //FAST_DELAY, NORMAL_DELAY, FILTERD_DELAY;
//
//    if (!LTC_readGPIOs((uint16_t*) batt->read_auxreg)) {
//    	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
//            uint16_t data = batt->read_auxreg[dev_idx * NUM_AUX_GROUP];
//            ADC_To_Pressure(dev_idx, batt->pressure, data);
//    	}
//    }
//}
//
//void Read_Atmos_Temp(ModuleData *mod) {
//	LTC_SPI_writeCommunicationSetting(NUM_MOD, BMS_MUX[14]);
//	LTC_SPI_requestData(2);
//
//	LTC_startADC_GPIO(MD_NORMAL, 1); //ADC mode: MD_FILTERED, MD_NORMAL, MD_FAST
//    HAL_Delay(NORMAL_DELAY); //FAST_DELAY, NORMAL_DELAY, FILTERD_DELAY;
//
//    if (!LTC_readGPIOs((uint16_t*) batt->read_auxreg)) {
//
//    	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
//
//            uint16_t data = batt->read_auxreg[dev_idx * NUM_AUX_GROUP];
//            Atmos_Temp_To_Celsius(dev_idx, batt->atmos_temp, data);
//
//    	}
//    }
//}
//
//
//// this function calculates dew point from the properties of a battery module object
//// Dew Point: temperature (in Celsius) that air needs to be cooled to reach 100% humidity
//void Get_Dew_Point(ModuleData *mod) {
//
//	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
//
//		uint16_t humidity = mod[dev_idx].humidity;
//		uint16_t atmos_temp = mod[dev_idx].atmos_temp;
//
//		// simple approximation that is accurate to within 1 deg C
//		// Only works well when Relative Humidity is above 50%
//
//		mod[dev_idx].dew_point = atmos_temp - ((float)(100 - humidity) / 5);
//
//	}
//
//}

//void Read_Humidity(ModuleData *mod) {
//	LTC_SPI_writeCommunicationSetting(NUM_MOD, BMS_MUX[13]);
//	LTC_SPI_requestData(2);
//
//	LTC_startADC_GPIO(MD_NORMAL, 1); //ADC mode: MD_FILTERED, MD_NORMAL, MD_FAST
//    HAL_Delay(NORMAL_DELAY); //FAST_DELAY, NORMAL_DELAY, FILTERD_DELAY;
//
//    if (!LTC_readGPIOs((uint16_t*) batt->read_auxreg)) {
//    	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
//        uint16_t data = batt->read_auxreg[dev_idx * NUM_AUX_GROUP];
//        ADC_To_Humidity(dev_idx, batt->humidity, data);
//    	}
//    }
//}




