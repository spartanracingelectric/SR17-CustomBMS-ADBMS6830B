/**
 * @file    safety.h
 * @brief   Public interface for BMS safety checks, thresholds, and status bits.
 *
 * This header collects:
 *  - Pack/cell voltage and temperature thresholds for FAULT and WARNING states
 *  - Latching margins (hysteresis) to clear fault/warning conditions
 *  - Bit masks for warning/fault summary bytes
 *  - Public APIs to evaluate cell/pack voltage, balancing, and temperatures, and
 *    to compute module-level averages
 *
 * Conventions:
 *  - Voltage thresholds are expressed in deci-millivolts (0.1 mV units).
 *      e.g., 4_100_000 → 410_000.0 mV = 410.000 V pack
 *            42_500    →  4_250.0  mV =   4.250 V per cell
 *  - Temperature thresholds are in degrees Celsius (°C).
 *  - Summary bitfields (warnings/faults) use the masks defined below.
 */
#ifndef INC_SAFETY_H_
#define INC_SAFETY_H_

#include "main.h"

/* ===== Voltage & Temperature Thresholds (FAULT) ==============================
 * Units:
 *  - PACK_* and CELL_* voltage thresholds → 0.1 mV steps (deci-mV).
 *  - CELL_HIGH_TEMP_FAULT → °C.
 */
#define PACK_HIGH_VOLT_FAULT	    4100000
#define PACK_LOW_VOLT_FAULT         2880000
#define CELL_HIGH_VOLT_DISCONNECT	45000
#define CELL_HIGH_VOLT_FAULT	    42500
#define CELL_LOW_VOLT_FAULT		    25000
#define CELL_VOLT_IMBALANCE_FAULT   2000 //0.1 V
#define CELL_HIGH_TEMP_FAULT		70

/* ===== Voltage & Temperature Thresholds (WARNING) ============================
 * Units:
 *  - Voltage in 0.1 mV steps; temperature in °C.
 */
#define PACK_HIGH_VOLT_WARNING	    4085000
#define PACK_LOW_VOLT_WARNING       3000000
#define CELL_HIGH_VOLT_WARNING	    40000
#define CELL_LOW_VOLT_WARNING	    27000
#define CELL_VOLT_IMBALANCE_WARNING	1000 //0.05 V
#define CELL_HIGH_TEMP_WARNING		55
#define CELL_LOW_TEMP_WARNING		0

/* ===== Latching Margins (Hysteresis to Clear States) =========================
 * Apply these margins before clearing a latched condition to avoid chatter.
 * Units:
 *  - *_VOLT margins in 0.1 mV steps (e.g., 100 → 10 mV).
 *  - *_TEMP margins in °C.
 */
#define FAULT_LOCK_MARGIN_HIGH_VOLT 100			//10 mV
#define FAULT_LOCK_MARGIN_LOW_VOLT 	1000		//100 mV
#define FAULT_LOCK_MARGIN_IMBALANCE 1000		//100 mV
#define FAULT_LOCK_MARGIN_HIGH_TEMP 10			//10 ℃

/* ===== Warning/Fault Summary Bit Masks ======================================
 * These masks map conditions into compact summary bytes for CAN/telemetry.
 * Bit positions are shown in comments for clarity.
 */
// Warning byte
//#define WARNING_BIT_SLAVE_VOLT 	(1 << 1) 	// 0b00000001 (Bit position 1)
#define WARNING_BIT_HIGH_TEMP 		(1 << 2) 	// 0b00000010 (Bit position 2)
#define WARNING_BIT_IMBALANCE       (1 << 3)  	// 0b00000100 (Bit position 3)
#define WARNING_BIT_LOW_VOLT     	(1 << 4)  	// 0b00001000 (Bit position 4)
#define WARNING_BIT_HIGH_VOLT    	(1 << 5)  	// 0b00010000 (Bit position 5)
#define WARNING_BIT_LOW_PACK_VOLT  	(1 << 6)	// 0b01000000 (Bit position 6)
#define WARNING_BIT_HIGH_PACK_VOLT 	(1 << 7)	// 0b10000000 (Bit position 7)

// Fault byte
#define FAULT_BIT_HIGH_TEMP 		(1 << 2) 	// 0b00000010 (Bit position 2)
#define FAULT_BIT_IMBALANCE       	(1 << 3)  	// 0b00000100 (Bit position 3)
#define FAULT_BIT_LOW_VOLT     		(1 << 4)  	// 0b00001000 (Bit position 4)
#define FAULT_BIT_HIGH_VOLT    		(1 << 5)  	// 0b00010000 (Bit position 5)
#define FAULT_BIT_LOW_PACK_VOLT  	(1 << 6)	// 0b01000000 (Bit position 6)
#define FAULT_BIT_HIGH_PACK_VOLT 	(1 << 7)	// 0b10000000 (Bit position 7)

/* ===== Additional Margins/Heuristics ========================================
 * SLAVE_VOLT_WARNING_MARGIN:
 *  - Per-module average voltage margin (0.1 mV units) for slave/module warnings.
 */
#define SLAVE_VOLT_WARNING_MARGIN 	100			//10 mV

/* ===== Public API: Safety Evaluators & Aggregates ============================
 * Cell_Voltage_Fault():
 *  - Evaluate per-cell over/under-voltage and pack limits; set warning/fault bits.
 *
 * Cell_Balance_Fault():
 *  - Evaluate cell imbalance relative to thresholds; set summary bits accordingly.
 *
 * Cell_Temperature_Fault():
 *  - Evaluate cell/board temperature vs. high/low thresholds (with margins).
 *
 * High_Voltage_Fault():
 *  - Evaluate pack-level HV sense vs. pack thresholds; set summary bits.
 *
 * Module_Voltage_Averages():
 *  - Compute per-module average voltages for diagnostic/telemetry use.
 *
 * Module_Temperature_Averages():
 *  - Compute per-module average temperatures.
 */
void Cell_Voltage_Fault(AccumulatorData *batt, ModuleData *mod, uint8_t *fault, uint8_t *warnings);
void Cell_Balance_Fault(AccumulatorData *batt, ModuleData *mod, uint8_t *fault, uint8_t *warnings);
void Cell_Temperature_Fault(AccumulatorData *batt, ModuleData *mod, uint8_t *fault, uint8_t *warnings);
void High_Voltage_Fault(AccumulatorData *batt, ModuleData *mod, uint8_t *fault, uint8_t *warnings);
void Module_Voltage_Averages(AccumulatorData *batt, ModuleData *mod);
void Module_Temperature_Averages(AccumulatorData *batt, ModuleData *mod);

#endif /* INC_SAFETY_H_ */
