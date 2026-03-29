#ifndef SHUNT_H_
#define SHUNT_H_
#include <stdint.h>
#include "accumulator.h"

typedef struct{
	int32_t current_mA;
	int64_t coulombCount;
}Shunt;

void Shunt_init(void);
void Shunt_handleCANMessage_Current(uint8_t rxData[8]);
void Shunt_handleCANMessage_Coulomb(uint8_t rxData[8]);
void Shunt_updateAccumulator(AccumulatorData* acc);

#endif
