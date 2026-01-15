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
#include <stdint.h>
#include <stdbool.h>

/* ===== Voltage & Temperature Thresholds (FAULT) ==============================
 * Units:
 *  - PACK_* and CELL_* voltage thresholds → 0.1 mV steps (deci-mV).
 *  - CELL_HIGH_TEMP_FAULT → °C.
 */
#define PACK_HIGH_VOLT_FAULT	    590000
#define PACK_LOW_VOLT_FAULT         288000
#define CELL_HIGH_VOLT_DISCONNECT	45000
#define CELL_HIGH_VOLT_FAULT	    4250
#define CELL_LOW_VOLT_FAULT		    2500
#define CELL_VOLT_IMBALANCE_FAULT   2000 //0.1 V
#define CELL_HIGH_TEMP_FAULT		70
#define CELL_LOW_TEMP_FAULT         0
#define CELL_OPEN_WIRE_FAULT        2000 
#define REDUNDANCY_VOLT_FAULT       5

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
#define FAULT_LOCK_MARGIN_TEMP  10			    //10 ℃
#define FAULT_LOCK_MARGIN_REDUNDANCY_VOLT 50	//5 mV

/* ===== Time Limits (Hysteresis to Assert States) =========================
 * Time limits (in ms) that a fault condition must persist before being asserted. 
 */
#define TIME_LIMIT_OVER_VOLT    500  
#define TIME_LIMIT_UNDER_VOLT   500  
#define TIME_LIMIT_OVER_TEMP    100  
#define TIME_LIMIT_UNDER_TEMP   100  
#define TIME_LIMIT_PEC          100
#define TIME_LIMIT_REDUNDANCY_VOLT 1000
#define TIME_LIMIT_OPEN_WIRE    100

/* ===== Additional Margins/Heuristics ========================================
 * SLAVE_VOLT_WARNING_MARGIN:
 *  - Per-module average voltage margin (0.1 mV units) for slave/module warnings.
 */
#define SLAVE_VOLT_WARNING_MARGIN 	100			//10 mV

/* ===== Warning/Fault Summary Bit Masks ======================================
 * These masks map conditions into compact summary bytes for CAN/telemetry.
 * Bit positions are shown in comments for clarity.
 */
// Warning byte
typedef struct WarningFlags_t {
    uint8_t OverVoltWarn     : 1;  
    uint8_t UnderVoltWarn    : 1;  
    uint8_t OverTempWarn     : 1;  
    uint8_t UnderTempWarn    : 1;  
    uint8_t ImbalanceWarn    : 1;  
    uint8_t Reserved         : 3;  
} WarningFlags_t;

// Fault byte
typedef struct FaultFlags_t {  
    uint8_t UnderVoltage     : 1;  
    uint8_t OpenWire         : 1;  
    uint8_t PEC              : 1;  
    uint8_t OverTemp         : 1;  
    uint8_t UnderTemp        : 1;
    uint8_t OverVoltage      : 1;
    uint8_t RedundancyVolt   : 1;  
    uint8_t RedundancyTemp   : 1;
} FaultFlags_t;

typedef enum FaultType_e {
    FAULT_NONE = 0,
    FAULT_OVER_VOLT,
    FAULT_UNDER_VOLT,
    FAULT_OPEN_WIRE,
    FAULT_PEC,
    FAULT_OVER_TEMP,
    FAULT_UNDER_TEMP,
    FAULT_REDUNDANT_VOLT,
    FAULT_REDUNDANT_TEMP
} FaultType_e;

typedef struct FaultMessage_t {
    uint8_t ModuleID;      // 0 to 9
    uint8_t CellID;        // 0 to 13
    FaultType_e FaultType; 
} FaultMessage_t;

extern FaultFlags_t   GlobalFaults[NUM_MOD][NUM_CELL_PER_MOD];
extern WarningFlags_t GlobalWarnings[NUM_MOD][NUM_CELL_PER_MOD];

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
void Safety_cellVoltageFault(AccumulatorData *batt, ModuleData *mod);
void Cell_Balance_Fault(AccumulatorData *batt, ModuleData *mod); 
void Safety_cellTemperatureFault(AccumulatorData *batt, ModuleData *mod);
void High_Voltage_Fault(AccumulatorData *batt, ModuleData *mod); 
void Module_Voltage_Averages(AccumulatorData *batt, ModuleData *mod); 
void Module_Temperature_Averages(AccumulatorData *batt, ModuleData *mod); 
bool Safety_getNextFault(FaultMessage_t *faultMsg);

#endif /* INC_SAFETY_H_ */
