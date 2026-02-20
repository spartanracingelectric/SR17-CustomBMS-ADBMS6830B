/**
 * @file    balance.h
 * @brief   Public interface for passive cell balancing for ADBMS6830b via DCC bits.
 */

#ifndef INC_BALANCE_H_
#define INC_BALANCE_H_

#include "module.h"
#include "pack.h"

typedef struct BalanceStatus
{
	uint8_t cellsToBalance[NUM_CELLS_PER_MODULE];
	uint16_t cellsBalancing;
} BalanceStatus;

typedef struct ConfigurationRegisterB
{
	uint16_t underVoltageThreshold_V;
	uint16_t overVoltageThreshold_V;
	uint16_t cellsDischargeStatus;
	// TODO: Add other fields in register
} ConfigurationRegisterB;

#define BALANCE_THRESHOLD_MV 50
#define BALANCE_COMMAND_TIMEOUT_MS 5000
#define BALANCE_COMMAND_CAN_ID 0x604

void Balance_init(BalanceStatus balanceStatus[NUM_MODULES_TOTAL], ConfigurationRegisterB configB[NUM_MODULES_TOTAL]);
void Balance_handleBalanceCANMessage(uint8_t rxData[8]);
void Balance_handleBalancing(ModuleData module[NUM_MODULES_TOTAL], PackData *pack, BalanceStatus balanceStatus[NUM_MODULES_TOTAL], ConfigurationRegisterB configB[NUM_MODULES_TOTAL]);
void Balance_setCellDischarge(ModuleData module[NUM_MODULES_TOTAL], PackData *pack, BalanceStatus balanceStatus[NUM_MODULES_TOTAL]);
void Balance_stopCellDischarge(BalanceStatus balanceStatus[NUM_MODULES_TOTAL]);
void Balance_getDischargeStatus(BalanceStatus balanceStatus[NUM_MODULES_TOTAL], ConfigurationRegisterB configB[NUM_MODULES_TOTAL]);

#endif
