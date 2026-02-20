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

#include <stdlib.h>

#include "main.h"
#include "pack.h"

/* ===== Shunt / Analog Front-End Parameters ===================================
 * MAX_SHUNT_AMPAGE:
 *  - Maximum measurable current magnitude permitted by the design/AFE.
 *
 * MAX_SHUNT_VOLTAGE:
 *  - Maximum voltage at the ADC/AFE input corresponding to full-scale current.
 *
 * SHUNT_R1 / SHUNT_R2 / SHUNT_OPAMP_RATIO:
 *  - Resistor values that establish the amplifier/divider gain from shunt to ADC.
 *  - SHUNT_OPAMP_RATIO = SHUNT_R1 / SHUNT_R2 (dimensionless gain factor).
 *
 * SHUNT_OFFSET:
 *  - Static bias used to null residual offset of the shunt/op-amp/ADC path.
 */
#define MAX_SHUNT_AMPAGE 500000
#define MAX_SHUNT_VOLTAGE 2.62f

#define SHUNT_R1 24925.0f  // 24.9 kΩ
#define SHUNT_R2 20000.0f  // 20 kΩ
#define SHUNT_OPAMP_RATIO SHUNT_R1 / SHUNT_R2
#define SHUNT_OFFSET 500  // 500 mA

/* ===== Public API ============================================================
 * SOC_getInitialCharge():
 *  - Estimate initial SoC from present pack/cell information at startup and
 *    seed batt->soc (project-defined scaling).
 *
 * SOC_updateCharge():
 *  - Update the SoC accumulator by integrating measured current over time.
 *    The elapsed_time argument is in milliseconds unless otherwise documented.
 */
void SOC_getInitialCharge(PackData *pack, ModuleData *mod);
void SOC_updateCharge(PackData *pack, uint32_t elapsed_time);

#endif
