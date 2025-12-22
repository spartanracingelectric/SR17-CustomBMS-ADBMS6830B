/**
 * @file    hv_sense.h
 * @brief   Public interface for high-voltage pack sensing and aggregation.
 *
 * This header collects:
 *  - Analog front-end constants (op-amp gain, divider values, ratio macro)
 *  - Public APIs to read/update HV input and to compute total pack voltage
 *
 * Conventions:
 *  - Voltages are expressed in millivolts (mV) unless otherwise noted.
 *  - The divider ratio uses R1 (high-side) and R2 (low-side): Vout = Vin * (R2 / (R1 + R2)).
 *  - GAIN_TLV9001 represents the post-divider amplification applied by the TLV9001 stage.
 *  - Functions write results into AccumulatorData and/or use ModuleData as sources.
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
#define GAIN_TLV9001 1.58f 		//TLV9001 gain
//Resistor values for the voltage divider
#define R1 1400000.0f  			//1.4 MΩ
#define R2 6761.5f     			//6863kΩ is the spec, but it might be parallel to other components. measured value was 6660kΩ, but it was too low, so I took average of them and it works well.
#define DIVIDER_RATIO (R1 + R2) / R2

/* ===== Public API ============================================================
 * ReadHVInput():
 *  - Samples the high-voltage sense path and updates batt->hvsens_pack_voltage
 *    (and/or related fields) after compensating for DIVIDER_RATIO and GAIN_TLV9001.
 *
 * getSumPackVoltage():
 *  - Aggregates per-cell voltages from ModuleData into batt->sum_pack_voltage.
 *  - Does not perform HV-sense measurement; it purely sums cell readings.
 */
void HVSense_getPackVoltage(AccumulatorData *batt);

#endif /* INC_HV_SENSE_H_ */
