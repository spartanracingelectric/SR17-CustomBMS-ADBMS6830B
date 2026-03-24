#include "charger.h"
#include "main.h"
#include "accumulator.h"

bool isCharging;
static AccumulatorData *accumulatorPointer = NULL;

void Charger_init(AccumulatorData *acc)
{
	isCharging = false;
    accumulatorPointer = acc;
}

void Charger_handleElconCANMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
	uint16_t rawCurrent = ((uint16_t)rxData[2] << 8) | rxData[3];
	int32_t current_mA = (int32_t)rawCurrent * 100;
	if (current_mA > 0)
	{
		isCharging = true;
	}
	accumulatorPointer->current_mA = current_mA;
}

// TODO: Add function to pass in SOC value from charger
