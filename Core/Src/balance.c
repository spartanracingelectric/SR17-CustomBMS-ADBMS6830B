/**
 * @file    balance.c
 * @brief   Passive cell balancing control for ADBM6830b via DCC bits.
 */

#include "balance.h"
#include "module.h"
#include "stm32f1xx_hal.h"
#include <adbms6830b.h>

bool isBalancingEnabled = false;
bool isBalancingFinished = false;
static uint32_t lastBalanceCommandTick = 0;

void Balance_init(BalanceStatus *balanceStatus, ConfigurationRegisterB *configB)
{
	isBalancingEnabled = false;
	isBalancingFinished = false;
	lastBalanceCommandTick = HAL_GetTick();
	Balance_stopCellDischarge(balanceStatus);
	Balance_getDischargeStatus(balanceStatus, configB);
}

// TODO: Move to can.c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	static CAN_RxHeaderTypeDef rxHeader;
	static uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
	{
		if (rxHeader.StdId == BALANCE_COMMAND_CAN_ID)
		{
			Balance_handleBalanceCANMessage(rxData);
		}
	}
}

void Balance_handleBalanceCANMessage(uint8_t rxData[])
{
	uint8_t balanceCommand = rxData[0];
	lastBalanceCommandTick = HAL_GetTick();
	if (balanceCommand == 1)
	{
		isBalancingEnabled = true;
	}
	else if (balanceCommand == 0)
	{
		isBalancingEnabled = false;
		isBalancingFinished = true;
	}
}

void Balance_handleBalancing(ModuleData *module, PackData *pack, BalanceStatus *balanceStatus, ConfigurationRegisterB *configB)
{
	// Stop balancing if balance command is not received every BALANCE_COMMAND_TIMEOUT_MS
	uint32_t currentTick = HAL_GetTick();
	if (isBalancingEnabled && (currentTick - lastBalanceCommandTick > BALANCE_COMMAND_TIMEOUT_MS))
	{
		isBalancingEnabled = false;
		isBalancingFinished = true;
	}

	if (isBalancingEnabled)
	{
		Balance_setCellDischarge(module, pack, balanceStatus);
		Balance_getDischargeStatus(balanceStatus, configB);
	}
	else if (isBalancingFinished)
	{
		// Runs once when balancing stops
		Balance_stopCellDischarge(balanceStatus);
		Balance_getDischargeStatus(balanceStatus, configB);
		isBalancingFinished = false;
	}
}

/* Discharge Algorithm
 *  For each cell, compare its voltage to lowest cell voltage pack.
 *  If the difference is greater than BALANCE_THRESHOLD_MV, enable discharge for that cell (DCC = 1)
 *  Discharging will only occur if Enable Balance CAN message is receieved every BALANCE_COMMAND_TIMEOUT_MS
 */
void Balance_setCellDischarge(ModuleData *module, PackData *pack, BalanceStatus *balanceStatus)
{
	for (uint8_t moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (uint8_t cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] - pack->lowestCellVoltage_mV > BALANCE_THRESHOLD_MV)
			{
				balanceStatus[moduleIndex].cellsToBalance[cellIndex] = 1;
			}
			else
			{
				balanceStatus[moduleIndex].cellsToBalance[cellIndex] = 0;
			}
		}
	}
	ADBMS_writeConfigurationRegisterB(balanceStatus);
}

void Balance_stopCellDischarge(BalanceStatus *balanceStatus)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			balanceStatus[moduleIndex].cellsToBalance[cellIndex] = 0;
		}
	}
	ADBMS_writeConfigurationRegisterB(balanceStatus);
}

void Balance_getDischargeStatus(BalanceStatus *balanceStatus, ConfigurationRegisterB *configB)
{
	ADBMS_readConfigurationRegisterB(configB);
	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		balanceStatus[moduleIndex].cellsBalancing = configB[moduleIndex].cellsDischargeStatus;
	}
}
