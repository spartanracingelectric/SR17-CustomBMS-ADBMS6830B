/**
 * @file    hv_sense.h
 */
#ifndef INC_HV_SENSE_H_
#define INC_HV_SENSE_H_

#include "main.h"

/* ===== Analog Front-End Parameters ==========================================
 * GAIN_TLV9001:
 *  - Voltage gain of the TLV9001 op-amp stage applied after the resistor divider.
 *
 * Divider network:
 *  - R1: upper (pack side → sense node) resistor in ohms.
 *  - R2: lower (sense node → ground) resistor in ohms.
 *  - DIVIDER_RATIO = (R1 + R2) / R2  (i.e., Vin/Vout when no op-amp gain is applied).
 *
 * Notes:
 *  - R2 comment documents spec vs. measured variance; keep consistent with PCB BOM.
 *  - Effective reconstruction of pack voltage typically uses: Vin ≈ Vmeas * DIVIDER_RATIO / GAIN_TLV9001.
 */

#define GAIN_TLV9001 1.58f

// Resistor values for the voltage divider
#define R1 1400000.0f
#define R2 4669.0f
#define DIVIDER_RATIO (R1 + R2) / R2
#define MULTIPLIER 317.460317

void HVSense_getPackVoltage(AccumulatorData *batt);

#endif
