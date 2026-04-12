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
#include "accumulator.h"
#include "module.h"
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
#define CELL_HIGH_VOLT_FAULT	    4200
#define CELL_LOW_VOLT_FAULT		    2800
#define CELL_VOLT_IMBALANCE_FAULT   2000 //0.1 V
#define CELL_HIGH_TEMP_FAULT		60
#define CELL_LOW_TEMP_FAULT         -1 // Temporary until sensor breakouts are fixed
#define CELL_OPEN_WIRE_FAULT        2000 
#define REDUNDANCY_VOLT_FAULT       5

/* ===== Voltage & Temperature Thresholds (WARNING) ============================
 * Units:
 *  - Voltage in 0.1 mV steps; temperature in °C.
 */
#define PACK_HIGH_VOLT_WARNING	    4085000
#define PACK_LOW_VOLT_WARNING       3000000
#define CELL_HIGH_VOLT_WARNING	    4000
#define CELL_LOW_VOLT_WARNING	    2700
#define CELL_VOLT_IMBALANCE_WARNING	1000 //0.05 V
#define CELL_HIGH_TEMP_WARNING		55
#define CELL_LOW_TEMP_WARNING		0

/* ===== Latching Margins (Hysteresis to Clear States) =========================
 * Apply these margins before clearing a latched condition to avoid chatter.
 * Units:
 *  - *_VOLT margins in 0.1 mV steps (e.g., 100 → 10 mV).
 *  - *_TEMP margins in °C.
 */
#define FAULT_LOCK_MARGIN_HIGH_VOLT 10			//10 mV
#define FAULT_LOCK_MARGIN_LOW_VOLT 	10		    //10 mV
#define FAULT_LOCK_MARGIN_IMBALANCE 100			//100 mV
#define FAULT_LOCK_MARGIN_TEMP  1			    //1 C
#define FAULT_LOCK_MARGIN_REDUNDANCY_VOLT 5 	//5 mV

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

typedef enum WarningType_e {
    WARNING_NONE            = 0x00,  // 0b00000000
    WARNING_OVER_VOLT       = 0x01,  // 0b00000001
    WARNING_UNDER_VOLT      = 0x02,  // 0b00000010
    WARNING_OVER_TEMP       = 0x04,  // 0b00000100
    WARNING_UNDER_TEMP      = 0x08,  // 0b00001000
    WARNING_IMBALANCE       = 0x10   // 0b00010000
} WarningType_e;

// Fault byte
typedef struct FaultFlags_t {  
    uint8_t UnderVolt        : 1;  
    uint8_t OpenWire         : 1;  
    uint8_t PEC              : 1;  
    uint8_t OverTemp         : 1;  
    uint8_t UnderTemp        : 1;
    uint8_t OverVolt         : 1;
    uint8_t RedundancyVolt   : 1;  
    uint8_t RedundancyTemp   : 1;
} FaultFlags_t;

typedef enum FaultType_e {
    FAULT_NONE             = 0x00,  // 0b00000000
    FAULT_OVER_VOLT        = 0x01,  // 0b00000001
    FAULT_UNDER_VOLT       = 0x02,  // 0b00000010
    FAULT_OPEN_WIRE        = 0x04,  // 0b00000100
    FAULT_PEC              = 0x08,  // 0b00001000
    FAULT_OVER_TEMP        = 0x10,  // 0b00010000
    FAULT_UNDER_TEMP       = 0x20,  // 0b00100000
    FAULT_REDUNDANT_VOLT   = 0x40,  // 0b01000000
    FAULT_REDUNDANT_TEMP   = 0x80   // 0b10000000
} FaultType_e;

typedef struct FaultMessage_t {
    uint8_t ModuleID;      // 0 to 9
    uint8_t CellID;        // 0 to 13
    uint8_t FaultType; 
} FaultMessage_t;

typedef struct WarningMessage_t {
    uint8_t ModuleID;     
    uint8_t CellID;
    uint8_t WarningType; 
} WarningMessage_t;

extern FaultFlags_t   GlobalFaults[NUM_MOD][NUM_CELL_PER_MOD];
extern WarningFlags_t GlobalWarnings[NUM_MOD][NUM_CELL_PER_MOD];
extern bool isFaulting;

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
void Safety_checkFaults(AccumulatorData *batt, ModuleData *mod);
void Safety_checkCellVoltageFault(AccumulatorData *batt, ModuleData *mod);
void Cell_Balance_Fault(AccumulatorData *batt, ModuleData *mod); 
void Safety_checkCellTemperatureFault(AccumulatorData *batt, ModuleData *mod);
void High_Voltage_Fault(AccumulatorData *batt, ModuleData *mod); 
void Module_Voltage_Averages(AccumulatorData *batt, ModuleData *mod); 
void Module_Temperature_Averages(AccumulatorData *batt, ModuleData *mod); 
bool Safety_getNextFault(FaultMessage_t *faultMsg);
bool Safety_getNextWarning(WarningMessage_t *warningMsg);
void Safety_getModuleFaultBits(uint16_t *faultBuffer);

#endif /* INC_SAFETY_H_ */
