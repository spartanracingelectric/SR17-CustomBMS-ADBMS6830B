/**
 * @file    safety.c
 * @brief   Pack/cell safety evaluation: voltage & temperature checks with hysteresis.
 *
 * This file provides:
 *  - Cell_Voltage_Fault(): per-cell UV/OV evaluation, warning/fault latching,
 *                          and fault line assertion/clear with hysteresis.
 *  - Cell_Temperature_Fault(): per-sensor high-temp evaluation with hysteresis.
 *  - (commented examples) Pack-level checks and module averages.
 *
 * Conventions:
 *  - Cell voltage units: deci-millivolts (0.1 mV). Example: 42_000 → 4.200 V.
 *  - Pack voltage fields used elsewhere (e.g., hvsens_pack_voltage) are centivolts (cV).
 *  - Temperatures are in degrees Celsius (°C).
 *  - Warning/fault summary bits are defined in safety.h.
 *
 * Hysteresis & Latching:
 *  - *_hysteresis counters debounce conditions across multiple cycles
 *    (e.g., >16) before asserting a fault bit.
 *  - *_fault_lock flags keep faults latched until the signal clears by the
 *    specified margin (FAULT_LOCK_MARGIN_*), at which point ClearFaultSignal()
 *    is invoked and bits are deasserted.
 *
 * Assumptions:
 *  - SendFaultSignal()/ClearFaultSignal() drive an external shutdown/fault line.
 *  - NUM_CELLS / NUM_THERM_TOTAL and thresholds are defined in headers.
 */
#include "safety.h"
#include "main.h"
#include "stdio.h"
#include "gpio.h"
#include "accumulator.h"

// ! Fault Thresholds

/* ===== Global Latches / Hysteresis Counters =================================
 * These persist across calls to provide debounce and latch behavior for
 * over-voltage, under-voltage, imbalance, and temperature conditions.
 */
uint8_t high_volt_fault_lock = 0;
uint8_t high_volt_hysteresis = 0;
uint8_t low_volt_hysteresis = 0;
uint8_t low_volt_fault_lock = 0;
uint8_t cell_imbalance_hysteresis = 0;
uint8_t high_temp_hysteresis = 0;

/* ===== Cell Voltage Evaluation ==============================================
 * Cell_Voltage_Fault():
 *  - Scans all cells to determine highest and lowest voltages.
 *  - Applies WARN thresholds for OV/UV.
 *  - Applies FAULT thresholds with counter-based hysteresis; asserts a latched
 *    fault and calls SendFaultSignal() once the counter exceeds its threshold.
 *  - Clears/relaxes a latched fault only after the measurement crosses back by
 *    the configured margin (FAULT_LOCK_MARGIN_*), then calls ClearFaultSignal().
 */
void Cell_Voltage_Fault(AccumulatorData *acc, ModuleData *mod, uint8_t *fault, uint8_t *warnings){
//high cell volt warning
		if (acc->cell_volt_highest >= CELL_HIGH_VOLT_WARNING) {
			*warnings |= WARNING_BIT_HIGH_VOLT;
		}
//high cell volt fault
		if ((acc->cell_volt_highest >= CELL_HIGH_VOLT_FAULT)) {
			if (high_volt_hysteresis > 16) {  //takes 50 cycle to fault
				high_volt_fault_lock = 1;
				*warnings &= ~WARNING_BIT_HIGH_VOLT;
				*fault |= FAULT_BIT_HIGH_VOLT;
				SendFaultSignal();
			}
			else {
				high_volt_hysteresis++;
			}
//			printf("high voltage fault signal on\n");
		}
//reset high cell volt fault
		else if (((acc->cell_volt_highest < (CELL_HIGH_VOLT_FAULT - FAULT_LOCK_MARGIN_HIGH_VOLT)) || (acc->cell_volt_highest > CELL_HIGH_VOLT_DISCONNECT))&& high_volt_fault_lock == 1){
			if (high_volt_hysteresis > 0){
				high_volt_hysteresis = 0;
				high_volt_fault_lock = 0;
				*warnings &= ~WARNING_BIT_HIGH_VOLT;
				*fault &= ~FAULT_BIT_HIGH_VOLT;
				ClearFaultSignal();
			}
		}

//low cell volt warning
		if (acc->cell_volt_lowest <= CELL_LOW_VOLT_WARNING) {
			*warnings |= WARNING_BIT_LOW_VOLT;
		}
//low cell volt fault
		if (acc->cell_volt_lowest <= CELL_LOW_VOLT_FAULT){
			if (low_volt_hysteresis > 16) {
				low_volt_fault_lock = 1;
				*warnings &= ~WARNING_BIT_LOW_VOLT;
				*fault |= FAULT_BIT_LOW_VOLT;
				SendFaultSignal();
			} else {
				low_volt_hysteresis++;
			}
//reset low cell volt fault
		} else if (acc->cell_volt_lowest > (CELL_LOW_VOLT_FAULT + FAULT_LOCK_MARGIN_LOW_VOLT)) {
			if (low_volt_hysteresis > 0) {
				low_volt_hysteresis = 0;
				*warnings &= ~WARNING_BIT_LOW_VOLT;
				*fault &= ~FAULT_BIT_LOW_VOLT;
				ClearFaultSignal();
			}
		}
}
//void Cell_Balance_Fault(struct batteryModule *batt, uint8_t *fault, uint8_t *warnings) {
//	batt->cell_difference = batt->cell_volt_highest - batt->cell_volt_lowest;
////cell volt imbalance warning
//	if (batt->cell_difference >= CELL_VOLT_IMBALANCE_WARNING) {
//		*warnings |= WARNING_BIT_IMBALANCE;
//	}
//}

/* ===== Cell Temperature Evaluation ==========================================
 * Cell_Temperature_Fault():
 *  - Scans all thermistor readings to find highest/lowest temperatures.
 *  - Sets warning when above CELL_HIGH_TEMP_WARNING but below FAULT threshold.
 *  - Uses a small hysteresis counter (e.g., >2) to assert a latched high-temp
 *    fault and SendFaultSignal(); clears once temperature falls below the
 *    threshold by FAULT_LOCK_MARGIN_HIGH_TEMP and calls ClearFaultSignal().
 */
void Cell_Temperature_Fault(AccumulatorData *batt, ModuleData *mod, uint8_t *fault, uint8_t *warnings) {
	batt->cell_temp_highest = mod->gpio_volt[0];
	batt->cell_temp_lowest = mod->gpio_volt[0];

	for (int i = 0; i < NUM_THERM_TOTAL; i++) {
		//find highest temp
		if (batt->cell_temp_highest < mod->gpio_volt[i]) {
			batt->cell_temp_highest = mod->gpio_volt[i];
		}
		if (batt->cell_temp_lowest > mod->gpio_volt[i]) {
			batt->cell_temp_lowest = mod->gpio_volt[i];
		}
	}

	//highest cell temp warning
	if (batt->cell_temp_highest >= CELL_HIGH_TEMP_WARNING && batt->cell_temp_highest < CELL_HIGH_TEMP_FAULT) {
		*warnings |= WARNING_BIT_HIGH_TEMP;
	}
	//highest cell temp fault

	if (batt->cell_temp_highest >= CELL_HIGH_TEMP_FAULT) {
		if (high_temp_hysteresis > 2) {
			*warnings &= ~WARNING_BIT_HIGH_TEMP;
			*fault |= FAULT_BIT_HIGH_TEMP;
			SendFaultSignal();
		} else {
			(high_temp_hysteresis)++;
		}
	} else if (batt->cell_temp_highest < (CELL_HIGH_TEMP_FAULT - FAULT_LOCK_MARGIN_HIGH_TEMP)) {
		if (high_temp_hysteresis > 0) {
			high_temp_hysteresis = 0;
			*warnings &= ~WARNING_BIT_HIGH_TEMP;
			*fault &= ~FAULT_BIT_HIGH_TEMP;
			ClearFaultSignal();
		}
	}
}

/* ===== (Examples/Disabled) Pack-Level & Module Aggregates ====================
 * Illustrative routines for pack HV thresholds and per-module averages. These
 * remain commented until needed on your hardware/software path.
 */
//void High_Voltage_Fault(struct batteryModule *batt, uint8_t *fault, uint8_t *warnings){
//	uint32_t sum_voltage = 0;
//
//	for (int i = 0; i < NUM_CELLS; i++) {
//		 sum_voltage += (uint32_t)batt->cell_volt[i]; //get sum voltage
//	}
//	if ((sum_voltage - batt->pack_voltage) >= FAULT_LOCK_MARGIN_LOW_VOLT){
//		*warnings |= WARNING_BIT_SLAVE_VOLT;
//	}
//	if (batt->pack_voltage >= PACK_HIGH_VOLT_WARNING) {
//		*warnings |= WARNING_BIT_HIGH_PACK_VOLT;
//	}
//	if (batt->pack_voltage <= PACK_LOW_VOLT_WARNING) {
//		*warnings |= WARNING_BIT_LOW_PACK_VOLT;
//	}
//	if (batt->pack_voltage >= PACK_HIGH_VOLT_FAULT) {
//		*fault |= FAULT_BIT_HIGH_PACK_VOLT;
//		HAL_GPIO_WritePin(MCU_SHUTDOWN_SIGNAL_GPIO_Port, MCU_SHUTDOWN_SIGNAL_Pin, GPIO_PIN_SET);
//	}
//	else{
//		*fault &= ~FAULT_BIT_HIGH_PACK_VOLT;
//	}
//	if (batt->pack_voltage <= PACK_LOW_VOLT_FAULT) {
//		*fault |= FAULT_BIT_LOW_PACK_VOLT;
//		HAL_GPIO_WritePin(MCU_SHUTDOWN_SIGNAL_GPIO_Port, MCU_SHUTDOWN_SIGNAL_Pin, GPIO_PIN_SET);
//	}
//	else{
//		*fault &= ~FAULT_BIT_LOW_PACK_VOLT;
//	}
//}


//void Module_Voltage_Averages(struct batteryModule *batt) {
//    for (int i = 0; i < NUM_CELLS; i += NUM_CELL_PER_MOD) {
//        uint16_t volt_sum = 0;
//
//        for (int j = i; j < i + NUM_CELL_PER_MOD && j < NUM_CELLS; j++) {
//            volt_sum += batt->cell_volt[j];
//        }
//
//        uint16_t average = volt_sum / NUM_CELL_PER_MOD;
//
//        batt->average_volt[i / NUM_CELL_PER_MOD] = average;
//    }
//}
//
//
//void Module_Temperature_Averages(struct batteryModule *batt) {
//    for (int i = 0; i < NUM_THERM_TOTAL; i += NUM_THERM_PER_MOD) {
//        uint16_t temp_sum = 0;
//
//        for (int j = i; j < i + NUM_THERM_PER_MOD && j < NUM_THERM_TOTAL; j++) {
//            temp_sum += batt->cell_temp[j];
//        }
//
//        uint16_t average = temp_sum / NUM_THERM_PER_MOD;
//
//        batt->average_temp[i / NUM_THERM_PER_MOD] = average;
//    }
//}
