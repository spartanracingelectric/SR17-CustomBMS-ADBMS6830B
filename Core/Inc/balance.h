/**
 * @file    balance.h
 * @brief   Public interface for passive cell balancing via ADBMS/LTC CFG DCC bits.
 *
 * This header collects:
 *  - Balancing constants (thresholds in mV)
 *  - Public API for initialization, start/stop control, and the discharge algorithm
 *  - Helper to pack DCC bits into per-device configuration bytes
 *
 * Conventions:
 *  - Voltage units are millivolts (mV) unless otherwise noted.
 *  - Balancing compares each cell against the lowest cell in the pack.
 *  - DCC bit mapping follows LTC/ADBMS CFG layout:
 *      * Cells 0–7   → CFG byte[4] bits 0–7
 *      * Cells 8–15  → CFG byte[5] bits 0–7
 *  - Arrays are sized using NUM_MOD / NUM_CELL_PER_MOD from main.h.
 */

#ifndef INC_BALANCE_H_
#define INC_BALANCE_H_

#include "main.h"

/* ===== Balancing Constants ===================================================
 * BALANCE_THRESHOLD:
 *  - If (cell_mV - lowest_cell_mV) > BALANCE_THRESHOLD, that cell is eligible
 *    for discharge (bleeding) via its DCC bit.
 */
#define BALANCE_THRESHOLD_mV 50 

/* ===== Public API: Lifecycle ================================================
 * Balance_init():
 *  - Clears all DCC states and internal flags, wakes isoSPI, and writes a
 *    default configuration (no balancing) to all devices.
 *
 * Start_Balance():
 *  - If balancing is enabled (set elsewhere), runs the discharge algorithm and
 *    writes the updated configuration (with DCC bits) to the chain.
 *
 * End_Balance():
 *  - When a finish flag is set (e.g., by CAN), clears all DCC bits and restores
 *    the default configuration on all devices.
 */
void Balance_init(BalanceStatus *blst, ConfigurationRegisterB *RDFCGB_buff);
void Balance_handleBalancing(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst, ConfigurationRegisterB *configB);
void Balance_setCellDischarge(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst);
void Balance_stopCellDischarge(BalanceStatus *bls);
void Balance_getDischargeStatus(BalanceStatus *blst, ConfigurationRegisterB *configB);


#endif /* INC_BALANCE_H_ */
