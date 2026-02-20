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

void Balance_init(BalanceStatus *balanceStatus, ConfigurationRegisterB *configB);
void Balance_handleBalancing(ModuleData *module, PackData *pack, BalanceStatus *balanceStatus, ConfigurationRegisterB *configB);
void Balance_setCellDischarge(ModuleData *module, PackData *pack, BalanceStatus *balanceStatus);
void Balance_stopCellDischarge(BalanceStatus *balanceStatus);
void Balance_getDischargeStatus(BalanceStatus *balanceStatus, ConfigurationRegisterB *configB);

#endif
