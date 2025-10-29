/*
 * accumulator.h
 *
 *  Created on: Oct 7, 2025
 *      Author: panda
 */

#ifndef INC_ACCUMULATOR_H_
#define INC_ACCUMULATOR_H_

#include "main.h"

void Accumulator_init(AccumulatorData *acc);
void Accumulator_getMinVolatage(AccumulatorData *acc, ModuleData *mod);
void Accumulator_getMaxVolatage(AccumulatorData *acc, ModuleData *mod);
void Accumulator_getTotalVoltage(AccumulatorData *batt, ModuleData *mod);

#endif /* INC_ACCUMULATOR_H_ */
