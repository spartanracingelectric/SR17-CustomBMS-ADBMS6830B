/**
 * @file    balance.h
 * @brief   Public interface for passive cell balancing for ADBMS6830b via DCC bits.
 */

#ifndef INC_BALANCE_H_
#define INC_BALANCE_H_

#include "main.h"

#define BALANCE_THRESHOLD_MV 50 
#define BALANCE_COMMAND_TIMEOUT_MS 5000
#define BALANCE_COMMAND_CAN_ID 0x604

void Balance_init(BalanceStatus *blst, ConfigurationRegisterB *RDFCGB_buff);
void Balance_handleBalancing(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst, ConfigurationRegisterB *configB);
void Balance_setCellDischarge(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst);
void Balance_stopCellDischarge(BalanceStatus *bls);
void Balance_getDischargeStatus(BalanceStatus *blst, ConfigurationRegisterB *configB);


#endif 
