/**
 * @file    balance.c
 * @brief   Passive cell balancing control for ADBM6830b via DCC bits.
 */
#include "balance.h"
#include "adbms6830b.h"
#include "safety.h"

BalanceState currentBalanceState = BALANCE_STATE_DISABLED;
BalanceState previousBalanceState = BALANCE_STATE_DISABLED;
static uint32_t lastBalanceCommandTick = 0;
static uint32_t balanceStateStartTick = 0;

void Balance_init(BalanceStatus *blst, ConfigurationRegisterB *configB)
{
	lastBalanceCommandTick = HAL_GetTick();
	balanceStateStartTick = HAL_GetTick();
	Balance_stopCellDischarge(blst);
	Balance_getDischargeStatus(blst, configB);
}

void Balance_handleBalanceCANMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
	uint8_t balanceCommand = rxData[0];
	lastBalanceCommandTick = HAL_GetTick();
	if (balanceCommand == 1 && currentBalanceState == BALANCE_STATE_DISABLED && !isFaulting)
	{
		currentBalanceState = BALANCE_STATE_ACTIVE;
		balanceStateStartTick = HAL_GetTick();
	}
	else if (balanceCommand == 0)
	{
		currentBalanceState = BALANCE_STATE_DISABLED;
		balanceStateStartTick = HAL_GetTick();
	}
}

void Balance_handleBalancing(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst, ConfigurationRegisterB *configB)
{
	// Stop balancing if balance command is not received every BALANCE_COMMAND_TIMEOUT_MS
	uint32_t currentTick = HAL_GetTick();
	if (currentBalanceState != BALANCE_STATE_DISABLED && (currentTick - lastBalanceCommandTick > BALANCE_COMMAND_TIMEOUT_MS))
	{
		currentBalanceState = BALANCE_STATE_DISABLED;
	}
	
	// Don't balance if there is a fault
	if (isFaulting) {
		currentBalanceState = BALANCE_STATE_DISABLED;
	}

	switch (currentBalanceState)
	{
	case BALANCE_STATE_DISABLED:
		if (previousBalanceState != BALANCE_STATE_DISABLED)
		{
			Balance_stopCellDischarge(blst);
			Balance_getDischargeStatus(blst, configB);
			previousBalanceState = BALANCE_STATE_DISABLED;
		}
		break;
	case BALANCE_STATE_REST:
		if (previousBalanceState != BALANCE_STATE_REST)
		{
			balanceStateStartTick = currentTick;
			Balance_stopCellDischarge(blst);
			Balance_getDischargeStatus(blst, configB);
			previousBalanceState = BALANCE_STATE_REST;
		}
		if (currentTick - balanceStateStartTick >= BALANCE_STATE_REST_LENGTH_MS)
		{
			previousBalanceState = BALANCE_STATE_REST;
			currentBalanceState = BALANCE_STATE_ACTIVE;
		}
		break;
	case BALANCE_STATE_ACTIVE:
		if (previousBalanceState != BALANCE_STATE_ACTIVE)
		{
			balanceStateStartTick = currentTick;
			Balance_setCellDischarge(mod, accm, blst);
			Balance_getDischargeStatus(blst, configB);
			previousBalanceState = currentBalanceState;
		}
		if (currentTick - balanceStateStartTick >= BALANCE_STATE_ACTIVE_LENGTH_MS)
		{
			previousBalanceState = BALANCE_STATE_ACTIVE;
			currentBalanceState = BALANCE_STATE_REST;
		}
		break;
	}
}

/* Discharge Algorithm
 *  For each cell, compare its voltage to lowest cell voltage pack.
 *  If the difference is greater than BALANCE_THRESHOLD_MV, enable discharge for that cell (DCC = 1)
 *  Discharging will only occur if Enable Balance CAN message is received every BALANCE_COMMAND_TIMEOUT_MS
 */
void Balance_setCellDischarge(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst)
{
	for (uint8_t moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		for (uint8_t cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
		{
			if (mod[moduleIndex].cellVoltage_mV[cellIndex] - accm->minCellVoltage_mV > BALANCE_THRESHOLD_MV)
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
