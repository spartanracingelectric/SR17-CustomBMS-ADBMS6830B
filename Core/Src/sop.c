/**
 * @file    sop.c
 * @brief   State-of-power (SoP) calculation of max feedback power from regenerative braking for the BMS.
 *
 * This file provides:
 *  - SOP_est(): calculates maximum safe feedback power to battery pack from regenerative braking
 *  - SOP_gt(): determines thermal conductance factor for thermal charge current from the cell with the 
 *    lowest temperature (permille i.e. 0.5 -> 500)
 *  - SOP_gz(): determines charge capacity conductance factor for thermal charge current from the current
 *    state of charge ratio (permille)
 *
 * Conventions:
 *  - Units:
 *      batt->sop : milliwatts (mW).
 *  - Initial SoP:
 *      * Set to zero
 * Notes:
 *  - COLD_STOP_CHARGE_TEMP_C, COLD_FULL_CHARGE_TEMP_C, HOT_FULL_CHARGE_TEMP_C, HOT_STOP_CHARGE_TEMP_C, 
 *    STATIC_INTERNAL_RESISTANCE_MO, MAX_PACK_VOLTAGE_V, NOMINAL_CHARGE_CURRENT_MA, LOW_RAMP_SOC, LOW_FULL_SOC,
 *    HIGH_RAMP_SOC, and HIGH_EDGE_SOC are defined in headers.
 */

#include "main.h"
#include "sop.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

//assumes temps stored as rounded integers
uint16_t SOP_gt(uint16_t cell_temp_lowest) {
    if (COLD_STOP_CHARGE_TEMP_C < cell_temp_lowest && cell_temp_lowest <= COLD_FULL_CHARGE_TEMP_C) {
        return 1000 * cell_temp_lowest / COLD_FULL_CHARGE_TEMP_C;
    }
    if (COLD_FULL_CHARGE_TEMP_C < cell_temp_lowest && cell_temp_lowest <= HOT_FULL_CHARGE_TEMP_C) {
        return 1000;
    }
    if (HOT_FULL_CHARGE_TEMP_C < cell_temp_lowest && cell_temp_lowest < HOT_STOP_CHARGE_TEMP_C) {
        return 1000 * (HOT_STOP_CHARGE_TEMP_C - cell_temp_lowest) / COLD_FULL_CHARGE_TEMP_C;
    }
    return 0;
} //currently exports permille

// soc currently not scaled properly for these calculations
// needs nominal_capacity to reduce to fraction form
uint32_t SOP_gz(uint32_t soc) {
    if (LOW_RAMP_SOC < soc && soc <= LOW_FULL_SOC) {
        return 1000*(soc - LOW_RAMP_SOC)/(LOW_FULL_SOC-LOW_RAMP_SOC);
    }
    if (LOW_FULL_SOC < soc && soc <= HIGH_RAMP_SOC) {
        return 1000;
    }
    if (HIGH_RAMP_SOC < soc && soc <= HIGH_EDGE_SOC) {
        return 1000*(HIGH_EDGE_SOC - soc)/(HIGH_EDGE_SOC-HIGH_RAMP_SOC);
    }
    return 0;
} //currently exports permille

// seemingly OCV via table is exported as per-cell
// need proper units/scaling principles for SoP calculation
uint32_t SOP_est(uint32_t soc, uint16_t cell_temp_lowest) {
    uint32_t ocv = OCV(soc, cell_temp_lowest) * NUM_CELLS_2; //placeholder function call (mV)
    uint32_t Ichgv = 1e3 * (MAX_PACK_VOLTAGE_MV - ocv) / STATIC_INTERNAL_RESISTANCE_MO; // (mA)
    Ichgv = Ichgv > 0 ? Ichgv : 0; //int max
    uint32_t Ichgt = SOP_gt(cell_temp_lowest) * SOP_gz(soc) * NOMINAL_CHARGE_CURRENT_MA / 1e6; // (mA)
    uint32_t Ichg = Ichgt > Ichgv? Ichgv : Ichgt; //int min
    uint32_t Vchg = ocv + Ichg * STATIC_INTERNAL_RESISTANCE_MO / 1e3; // (mV)
    return Vchg * Ichg / 1e3; // (mW)
}