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
static uint32_t Timer_OverVolt[NUM_MOD][NUM_CELL_PER_MOD];
static uint32_t Timer_UnderVolt[NUM_MOD][NUM_CELL_PER_MOD];
static uint32_t Timer_OverTemp[NUM_MOD][NUM_THERM_PER_MOD];
static uint32_t Timer_UnderTemp[NUM_MOD][NUM_THERM_PER_MOD];
static uint32_t Timer_PEC[NUM_MOD];

static FaultFlags_t   GlobalFaults[NUM_MOD][NUM_CELL_PER_MOD];
static WarningFlags_t GlobalWarnings[NUM_MOD][NUM_CELL_PER_MOD];

/* ===== Cell Voltage Evaluation ==============================================
 * Cell_Voltage_Fault():
 *  - Scans all cells to determine highest and lowest voltages.
 *  - Applies WARN thresholds for OV/UV.
 *  - Applies FAULT thresholds with counter-based hysteresis; asserts a latched
 *    fault and calls SendFaultSignal() once the counter exceeds its threshold.
 *  - Clears/relaxes a latched fault only after the measurement crosses back by
 *    the configured margin (FAULT_LOCK_MARGIN_*), then calls ClearFaultSignal().
 */
void Cell_Voltage_Fault(AccumulatorData *acc, ModuleData *mod){
	uint32_t current_time = HAL_GetTick();
    Accumulator_getMaxVoltage(acc, mod);
    Accumulator_getMinVoltage(acc, mod);

    for (int m = 0; m < NUM_MOD; m++) 
    {
        if (mod[m].pec_error_count > 0) 
        {
            if (Timer_PEC[m] == 0) {
                Timer_PEC[m] = current_time;
            }
             
            else if ((current_time - Timer_PEC[m]) > TIME_LIMIT_PEC) 
            {
                for (int c = 0; c < NUM_CELL_PER_MOD; c++) 
                {
                    GlobalFaults[m][c].PEC = 1;
                }

                SendFaultSignal();
            }

            continue;
        } 
        else 
        {
            Timer_PEC[m] = 0;
            for (int c = 0; c < NUM_CELL_PER_MOD; c++) 
            {
                GlobalFaults[m][c].PEC = 0;
            }

            ClearFaultSignal();
        }

        for (int c = 0; c < NUM_CELL_PER_MOD; c++) 
        {
            uint16_t voltage = mod[m].cell_volt[c];
            FaultFlags_t   *faults = &GlobalFaults[m][c];
            WarningFlags_t *warns  = &GlobalWarnings[m][c];

            if (voltage < CELL_OPEN_WIRE_FAULT) 
            {
                faults->OpenWire = 1;
                SendFaultSignal();
            } 
            else 
            {
                faults->OpenWire = 0;
            }

            int32_t diff = (int32_t)voltage - (int32_t)mod[m].redundantCellVoltage_mV[c];

            if (diff < 0) 
            {
                diff = -diff; 
            }

            if (diff > REDUNDANCY_VOLT_FAULT) 
            {
                faults->RedundancyVolt = 1;
                SendFaultSignal();
            } 
            else 
            {
                faults->RedundancyVolt = 0;
            }

            if (voltage >= CELL_HIGH_VOLT_WARNING) 
            {
                warns->OverVoltWarn = 1;
            }
            else 
            {
                warns->OverVoltWarn = 0;
            }

            if (voltage >= CELL_HIGH_VOLT_FAULT) 
            {
                if (Timer_OverVolt[m][c] == 0) 
                {
                    Timer_OverVolt[m][c] = current_time;
                } 
				
				else if ((current_time - Timer_OverVolt[m][c]) > TIME_LIMIT_OVER_VOLT) 
                {
                    faults->OverVoltage = 1;
                    SendFaultSignal(); 
                }
            } 
			else if (voltage < (CELL_HIGH_VOLT_FAULT - FAULT_LOCK_MARGIN_HIGH_VOLT)) 
            {
                Timer_OverVolt[m][c] = 0; 
                faults->OverVoltage = 0; 
                ClearFaultSignal();  
            }

            if (voltage <= CELL_LOW_VOLT_WARNING) 
            {
                warns->UnderVoltWarn = 1;
            }
            else 
            {
                warns->UnderVoltWarn = 0;
            }

            if (voltage <= CELL_LOW_VOLT_FAULT) 
            {
                if (Timer_UnderVolt[m][c] == 0) 
                {
                    Timer_UnderVolt[m][c] = current_time;
                } 
                else if ((current_time - Timer_UnderVolt[m][c]) > TIME_LIMIT_UNDER_VOLT) 
                {
                    faults->UnderVoltage = 1;
                    SendFaultSignal();
                }
            } 
            else if (voltage > (CELL_LOW_VOLT_FAULT + FAULT_LOCK_MARGIN_LOW_VOLT)) 
            {
                Timer_UnderVolt[m][c] = 0;
                faults->UnderVoltage = 0;
				ClearFaultSignal();
            }
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
void Cell_Temperature_Fault(AccumulatorData *batt, ModuleData *mod) {
	uint32_t current_time = HAL_GetTick();
    batt->cell_temp_highest = mod[0].pointTemp_C[0];
    batt->cell_temp_lowest  = mod[0].pointTemp_C[0];

    for (int m = 0; m < NUM_MOD; m++) 
    {
        for (int t = 0; t < NUM_THERM_PER_MOD; t++) 
        {
            uint16_t temp = mod[m].pointTemp_C[t];

            if (temp > batt->cell_temp_highest) batt->cell_temp_highest = temp;
            if (temp < batt->cell_temp_lowest)  batt->cell_temp_lowest  = temp;

            FaultFlags_t   *faults = &GlobalFaults[m][t]; 
            WarningFlags_t *warns  = &GlobalWarnings[m][t];

            if (temp >= CELL_HIGH_TEMP_WARNING) 
            {
                warns->OverTempWarn = 1;
            }
            else 
            {
                warns->OverTempWarn = 0;
            }

            if (temp >= CELL_HIGH_TEMP_FAULT) 
            {
                if (Timer_OverTemp[m][t] == 0) 
                {
                    Timer_OverTemp[m][t] = current_time;
                }
                else if ((current_time - Timer_OverTemp[m][t]) > TIME_LIMIT_OVER_TEMP) 
                {
                    faults->OverTemp = 1;
                    SendFaultSignal();
                }
            } 
            else if (temp < (CELL_HIGH_TEMP_FAULT - FAULT_LOCK_MARGIN_HIGH_TEMP)) 
            {
                Timer_OverTemp[m][t] = 0;
                faults->OverTemp = 0;
                ClearFaultSignal();
            }

            if (temp <= CELL_LOW_TEMP_WARNING) 
            {
                warns->UnderTempWarn = 1;
            }
			else 
            {
                warns->UnderTempWarn = 0;
            }

			if (temp <= CELL_LOW_TEMP_FAULT) 
            {
				if (Timer_UnderTemp[m][t] == 0) {
					Timer_UnderTemp[m][t] = current_time;
				}
				else if ((current_time - Timer_UnderTemp[m][t]) > TIME_LIMIT_UNDER_TEMP) 
                {
					faults->UnderTemp = 1;
					SendFaultSignal();
				}
			} 
			else if (temp > (CELL_LOW_TEMP_FAULT + FAULT_LOCK_MARGIN_HIGH_TEMP)) 
            {
				Timer_UnderTemp[m][t] = 0;
				faults->UnderTemp = 0;
                ClearFaultSignal();
			}
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
