/**
 * @file    balance.c
 * @brief   Passive cell balancing control for ADBMS/LTC68xx chains via CFG DCC bits.
 *
 * This module provides:
 *  - CAN-controlled enable/disable of balancing (StdId = 0x604, byte[0] = 1/0)
 *  - Discharge decision logic based on per-cell delta vs pack lowest cell
 *  - Packing/unpacking of DCC bits into CFG bytes for each device
 *  - Init/teardown helpers that write default or active CFG to all devices
 *
 * Conventions:
 *  - Balancing threshold is expressed in millivolts (see BALANCE_THRESHOLD).
 *  - DCC bit mapping follows LTC/ADBMS CFG layout:
 *      * Cells 0–7   → config[dev][4] bits 0–7
 *      * Cells 8–15  → config[dev][5] bits 0–7
 *  - isoSPI must be awake (Wakeup_Sleep / 0xFF tick) before any write command.
 *  - This file DOES NOT alter ADC sampling state; it only toggles DCC via CFG.
 */
#include <adbms6830b.h>
#include "balance.h"
#include "can.h"
#include <stdio.h>
#include "usart.h"

/* ===== Optional Defaults (kept as reference; not used at runtime) ============
 * These show typical fields found in LTC/ADBMS configuration registers.
 * They are commented out to avoid altering runtime behavior.
 */
// static int      GPIO[5]      = { 1, 1, 1, 1, 1 };
// static int      REFON        = 0;
// static int      DTEN         = 1; // READ-ONLY
// static int      ADCOPT       = 0;
// static uint8_t  VUV          = 0x00;
// static uint8_t  VOV_and_VUV  = 0x00;
// static uint8_t  VOV          = 0x00;
// static int      DCTO[4]      = { 1, 1, 1, 1 };

/* ===== CAN RX State ==========================================================
 * rxHeader/rxData buffer incoming frames. The charger command (StdId=0x604)
 * sets 'balance' (enable) or triggers 'balance_finish' (disable & reset).
 */
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
bool isBalancingEnabled = false;
bool isBalancingFinished = false;

/* ===== Public API: Initialization & Tear-down ================================
 * Balance_init():  Clear flags and DCCs, wake chain, write default CFG.
 * Start_Balance(): Run decision algorithm and push updated CFG (if enabled).
 * End_Balance():   If finish flag set, clear DCCs and restore default CFG.
 */
void Balance_init(BalanceStatus *blst, ConfigurationRegisterB *configB)
{
	isBalancingEnabled = false;
	isBalancingFinished = false;
	Balance_stopCellDischarge(blst);
	Balance_getDischargeStatus(blst, configB);
}

/* ===== Interrupt/Callback: CAN RX ===========================================
 * HAL callback for FIFO0 reception. Expects StdId=0x604 from charger:
 *  - data[0] == 1 → enable balancing
 *  - data[0] == 0 → disable balancing and request CFG reset
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) 
{
//    printf("fifo 0 callback\n");
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        if (rxHeader.StdId == 0x604) {  // CAN message from charger
            uint8_t balanceCommand = rxData[0]; // see the data bit on CAN

            // change the BALANCE flag to enable balance
            if (balanceCommand == 1) {
            	isBalancingEnabled = true;  // enable balance
//                printf("BALANCE enabled by CAN message.\n");
            } else if (balanceCommand == 0) {
            	isBalancingEnabled = false; 
            	isBalancingFinished = true;
//                printf("BALANCE disabled by CAN message.\n");
            }
        }
    }
}

/* ===== Control: Start / End ==================================================
 * Start_Balance(): Executes Discharge_Algo() and writes config when enabled.
 * End_Balance():   Clears DCC and writes defaultConfig when finish flag set.
 */

void Balance_handleBalancing(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst, ConfigurationRegisterB *configB)
{
	if (isBalancingEnabled)
	{
		Balance_setCellDischarge(mod, accm, blst);
		Balance_getDischargeStatus(blst, configB);
	}
	else if (isBalancingFinished) 
	{
		Balance_stopCellDischarge(blst);
		Balance_getDischargeStatus(blst, configB);
		isBalancingFinished = false;
	}
}


/* ===== Algorithm: Discharge Decision ========================================
 * Discharge_Algo():
 *  - For each device/cell, compare (cell_mV - lowest_mV) to BALANCE_THRESHOLD.
 *  - If above threshold → DCC=1 (enable bleed), else DCC=0.
 *  - balanceStatus bitmap mirrors the DCC decision per device.
 *
 * NOTE:
 *  - Local DCC array here is sized 12. If NUM_CELL_PER_MOD > 12 (e.g., 14),
 *    ensure your HW/chip variant’s DCC mapping matches this size choice.
 */
void Balance_setCellDischarge(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst) 
{
	for (uint8_t moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) {
		// check if each cell is close within 0.005V of the lowest cell.
		for (uint8_t cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++) 
		{
			if (mod[moduleIndex].cell_volt[cellIndex] - accm->cell_volt_lowest > BALANCE_THRESHOLD_mV) 
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

void Balance_getDischargeStatus(BalanceStatus *blst, ConfigurationRegisterB *configB) {
	ADBMS_readConfigurationRegisterB(configB);
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		blst[moduleIndex].cellsBalancing = configB[moduleIndex].cellsDischargeStatus; 
	}
}

