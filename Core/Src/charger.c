#include "charger.h"

bool isCharging;
static AccumulatorData *accumulatorPointer = NULL;
static int lastChargingMsgTime = 0;

void Charger_init(AccumulatorData *acc)
{
	isCharging = false;
    accumulatorPointer = acc;
}

void Charger_handleElconCANMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
	uint16_t rawCurrent = ((uint16_t)rxData[2] << 8) | rxData[3];
	int32_t current_mA = (int32_t)rawCurrent * 100;
	lastChargingMsgTime = HAL_GetTick();

	if (current_mA > 0)
	{
		isCharging = true;
	} 

	accumulatorPointer->chargerCurrent_mA = current_mA;
}

void Charger_updateStatus(AccumulatorData *acc) 
{
	static int currentTime = 0;
	currentTime = HAL_GetTick();

	if (acc->chargerCurrent_mA == 0 || currentTime - lastChargingMsgTime > 5000) 
	{
		isCharging = false;
	}
}

// TODO: Add function to pass in SOC value from charger
