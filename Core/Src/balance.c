/**
 * @file    balance.c
 * @brief   Passive cell balancing control for ADBM6830b via DCC bits.
 */

#include <adbms6830b.h>
#include "balance.h"
#include "can.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "usart.h"

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
bool isBalancingEnabled = false;
bool isBalancingFinished = false;
static uint32_t lastBalanceCommandTick = 0;

void Balance_init(BalanceStatus *blst, ConfigurationRegisterB *configB)
{
	isBalancingEnabled = false;
	isBalancingFinished = false;
    lastBalanceCommandTick = HAL_GetTick();
	Balance_stopCellDischarge(blst);
	Balance_getDischargeStatus(blst, configB);
}

// TODO: Move to can.c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) 
{
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) 
	{
		if (rxHeader.StdId == BALANCE_COMMAND_CAN_ID) 
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
	}
}

void Balance_handleBalancing(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst, ConfigurationRegisterB *configB)
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
		Balance_setCellDischarge(mod, accm, blst);
		Balance_getDischargeStatus(blst, configB);
	}
	else if (isBalancingFinished) 
	{
		// Runs once when balancing stops
		Balance_stopCellDischarge(blst);
		Balance_getDischargeStatus(blst, configB);
		isBalancingFinished = false;
	}
}


/* Discharge Algorithm
 *  For each cell, compare its voltage to lowest cell voltage pack. 
 *  If the difference is greater than BALANCE_THRESHOLD_MV, enable discharge for that cell (DCC = 1)
 *  Discharging will only occur if Enable Balance CAN message is receieved every BALANCE_COMMAND_TIMEOUT_MS
 */
void Balance_setCellDischarge(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst) 
{
	for (uint8_t moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		for (uint8_t cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++) 
		{
			if (mod[moduleIndex].cell_volt[cellIndex] - accm->cell_volt_lowest > BALANCE_THRESHOLD_MV) 
			{
				blst[moduleIndex].cellsToBalance[cellIndex] = 1;
			} 
			else
			{
				blst[moduleIndex].cellsToBalance[cellIndex] = 0;
			}
		}
	}	
	ADBMS_writeConfigurationRegisterB(blst);
}

void Balance_stopCellDischarge(BalanceStatus *blst)
{
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++) 
		{
			blst[moduleIndex].cellsToBalance[cellIndex] = 0;
		}
	}
	ADBMS_writeConfigurationRegisterB(blst);
}

void Balance_getDischargeStatus(BalanceStatus *blst, ConfigurationRegisterB *configB) 
{
	ADBMS_readConfigurationRegisterB(configB);
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		blst[moduleIndex].cellsBalancing = configB[moduleIndex].cellsDischargeStatus; 
	}
}

