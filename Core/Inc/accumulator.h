/*
 * accumulator.h
 *
 *  Created on: Oct 7, 2025
 *      Author: panda
 */

#ifndef INC_ACCUMULATOR_H_
#define INC_ACCUMULATOR_H_

#include "main.h"

void accumulator_init(AccumulatorData *acc);

void accumulator_getMinVolatage(AccumulatorData *acc, ModuleData *mod);

void accumulator_getMaxVolatage(AccumulatorData *acc, ModuleData *mod);

#endif /* INC_ACCUMULATOR_H_ */
