/**
 * @file    balance.h
 * @brief   Public interface for passive cell balancing for ADBMS6830b via DCC bits.
 */

#ifndef INC_BALANCE_H_
#define INC_BALANCE_H_

#include "accumulator.h"
#include "main.h"

typedef struct BalanceStatus
{
	uint8_t cellsToBalance[NUM_CELL_PER_MOD];
	uint16_t cellsBalancing;
} BalanceStatus;

typedef struct ConfigurationRegisterB
{
	uint16_t underVoltageThreshold_V;
	uint16_t overVoltageThreshold_V;
	uint16_t cellsDischargeStatus;
	// TODO: Add other fields in register
} ConfigurationRegisterB;

#define BALANCE_THRESHOLD_MV 5
#define BALANCE_COMMAND_TIMEOUT_MS 5000

void Balance_init(BalanceStatus *blst, ConfigurationRegisterB *RDFCGB_buff);
void Balance_handleBalancing(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst, ConfigurationRegisterB *configB);
void Balance_setCellDischarge(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst);
void Balance_stopCellDischarge(BalanceStatus *bls);
void Balance_getDischargeStatus(BalanceStatus *blst, ConfigurationRegisterB *configB);
void Balance_handleBalanceCANMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);

#endif
