/**
 * @file    soc.h
 * @brief   Public interface for SoC estimation (coulomb counting) and shunt sensing.
 *
 * This header collects:
 *  - Shunt/AFE constants (max measurable current/voltage, gain ratio, offset)
 *  - Public APIs to initialize SoC and to integrate current over elapsed time
 *
 * Conventions:
 *  - Current/charge units in AccumulatorData are project-defined (see main.h):
 *    the SoC accumulator is stored in a scaled integer (used later for % and
 *    milli-units by dividing in CAN formatting code).
 *  - SHUNT_OPAMP_RATIO expresses the front-end gain from the shunt to the ADC
 *    (derived from the resistor pair R1/R2 shown below).
 *  - MAX_SHUNT_VOLTAGE is the input range limit at the ADC/AFE after gain.
 *  - SHUNT_OFFSET is a static offset applied to compensate measurement bias
 *    (units are project-defined; typically mA).
 */
#ifndef INC_SOC_H_
#define INC_SOC_H_

#include "main.h"
#include "accumulator.h"
#include "module.h"

#define NUM_OCV_TABLES 3
#define MAX_CELL_CAPACITY_MAH 5000
#define PACK_CAPACITY_UAH (MAX_CELL_CAPACITY_MAH * NUM_CELLS_IN_PARALLEL * 1000)
#define SOC_SAVE_THRESHOLD_UAH (PACK_CAPACITY_UAH / 100)

/* ===== Public API ============================================================
 * SOC_getInitialCharge():
 *  - Estimate initial SoC from present pack/cell information at startup and
 *    seed batt->soc (project-defined scaling).
 *
 * SOC_updateCharge():
 *  - Update the SoC accumulator by integrating measured current over time.
 *    The elapsed_time argument is in milliseconds unless otherwise documented.
 */
void SOC_updateCharge(AccumulatorData *batt, uint32_t elapsed_time);
void SOC_init(AccumulatorData *batt, ModuleData *mod);
float SOC_getPercent(AccumulatorData *batt);

#endif
