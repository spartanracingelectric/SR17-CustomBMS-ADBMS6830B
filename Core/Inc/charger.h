#ifndef CHARGER_H
#define CHARGER_H

#include "accumulator.h"
#include "main.h"

extern bool isCharging;

void Charger_init(AccumulatorData *acc);
void Charger_handleElconCANMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);

#endif
