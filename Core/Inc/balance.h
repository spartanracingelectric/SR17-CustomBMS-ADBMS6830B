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
#define BALANCE_THRESHOLD 50 //mv

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
void Balance_init(BalanceStatus *blst, RDFCGB_buffer *RDFCGB_buff);
void Start_Balance(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst);
void End_Balance(BalanceStatus *blst, RDFCGB_buffer *RDFCGB_buff);

/* ===== Public API: Algorithm ================================================
 * Discharge_Algo():
 *  - For each device and cell, compares against the pack’s lowest cell.
 *  - Sets DCC=1 when (cell_mV - lowest_mV) > BALANCE_THRESHOLD; otherwise 0.
 *  - Mirrors decisions into balanceStatus bitfields per device.
 *
 * Parameters:
 *  - read_volt:      pointer to the contiguous array of per-cell voltages [mV],
 *                    of size NUM_MOD * NUM_CELL_PER_MOD.
 *  - lowest:         the lowest cell voltage [mV] across the pack (reference).
 *  - balanceStatus:  per-device bitfield array reflecting current DCC decisions;
 *                    element i holds the bitmask for device i.
 */
void Balance_setDCCbits(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst);

/* ===== Public API: Helpers ===============================================Balance_reset(BalanceStatus *blst)====
 * Balance_reset():
 *  - Clears all DCC decisions (sets all to 0) and zeroes balanceStatus.
 *
 * Set_Cfg():
 *  - Packs a device’s DCC bits into its configuration image:
 *      cells 0..7  → CFG[4] bit 0..7
 *      cells 8..15 → CFG[5] bit 0..7
 *
 * Parameters:
 *  - dev_idx:  index of the device in the daisy chain (0..NUM_MOD-1).
 *  - DCC:      pointer to per-cell discharge control bits (1=on, 0=off) for
 *              the given device; length must cover NUM_CELL_PER_MOD.
 */
void Balance_reset(BalanceStatus *blst, RDFCGB_buffer *RDFCGB_buff);

void Get_balanceStatus(BalanceStatus *blst, RDFCGB_buffer *rdfcgb);

#endif /* INC_BALANCE_H_ */
