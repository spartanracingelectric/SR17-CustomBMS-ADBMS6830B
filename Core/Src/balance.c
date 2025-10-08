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
uint8_t balance = 0;			//FALSE
uint8_t balance_finish = 0;

/* ===== Public API: Initialization & Tear-down ================================
 * Balance_init():  Clear flags and DCCs, wake chain, write default CFG.
 * Start_Balance(): Run decision algorithm and push updated CFG (if enabled).
 * End_Balance():   If finish flag set, clear DCCs and restore default CFG.
 */
void Balance_init(BalanceStatus *blst, RDFCGB_buffer *RDFCGB_buff){
	balance = 0;
	balance_finish = 0;
	Balance_reset(blst, RDFCGB_buff);
	Wakeup_Sleep();
	ADBMS_writeCFGB(blst);
}

/* ===== Interrupt/Callback: CAN RX ===========================================
 * HAL callback for FIFO0 reception. Expects StdId=0x604 from charger:
 *  - data[0] == 1 → enable balancing
 *  - data[0] == 0 → disable balancing and request CFG reset
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
//    printf("fifo 0 callback\n");
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        if (rxHeader.StdId == 0x604) {  // CAN message from charger
            uint8_t balanceCommand = rxData[0]; // see the data bit on CAN

            // change the BALANCE flag to enable balance
            if (balanceCommand == 1) {
            	balance = 1;  // enable balance
//                printf("BALANCE enabled by CAN message.\n");
            } else if (balanceCommand == 0) {
            	balance = 0;  // disable balance
            	balance_finish = 1;
//                printf("BALANCE disabled by CAN message.\n");
            }
        }
    }
}

/* ===== Control: Start / End ==================================================
 * Start_Balance(): Executes Discharge_Algo() and writes config when enabled.
 * End_Balance():   Clears DCC and writes defaultConfig when finish flag set.
 */
void Start_Balance(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst) {
//	printf("balance enable is %d\n", balance);
	if(balance > 0){
		Balance_setDCCbits(mod, accm , blst);
		Wakeup_Sleep();
		ADBMS_writeCFGB(blst);
	}
	else{
		return;
	}
}

void End_Balance(BalanceStatus *blst, RDFCGB_buffer *RDFCGB_buff) {
	if(balance_finish == 1){
		Balance_reset(blst, RDFCGB_buff);
		Wakeup_Sleep();
		ADBMS_writeCFGB(blst);
		balance_finish = 0;
	}
	else{
		return;
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
void Balance_setDCCbits(ModuleData *mod, AccumulatorData *accm, BalanceStatus *blst) {
	for (uint8_t modIndex = 0; modIndex < NUM_MOD; modIndex++) {
		// check if each cell is close within 0.005V of the lowest cell.
		for (uint8_t cell_idx = 0; cell_idx < NUM_CELL_PER_MOD; cell_idx++) {
			if (mod[modIndex].cell_volt[cell_idx] - accm->cell_volt_lowest > BALANCE_THRESHOLD) {
				blst[modIndex].balance_cells[cell_idx] = 1;
			} else{
				blst[modIndex].balance_cells[cell_idx] = 0;
			}
//			printf("M%d, cell: %d, diff: %d\n", modIndex + 1, cell_idx + 1, mod[0].cell_volt[cell_idx] - accm->cell_volt_lowest);
//			printf("BALANCE_THRESHOLD: %d\n", BALANCE_THRESHOLD);
		}
	}
}

/* ===== Helpers: Clear DCC / Apply CFG =======================================
 * Balance_reset(): Clear all status bits and set DCC=0 for every cell/device.
 * Set_Cfg():       Map DCC array into config[dev][4] (cells 0–7) and
 *                  config[dev][5] (cells 8–15) bit positions.
 */
void Balance_reset(BalanceStatus *blst, RDFCGB_buffer *RDFCGB_buff) {
	 for (int dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
	        for (int cell_idx = 0; cell_idx < NUM_CELL_PER_MOD; cell_idx++) {
	            blst[dev_idx].balance_cells[cell_idx] = 0;
	            blst[dev_idx].balancing_cells = 0xFFFF;
	        }
	        for(int cfgIndex = 0; cfgIndex < 6; cfgIndex++){
	        	RDFCGB_buff[dev_idx].CFGBR[cfgIndex] = 0x00;
	        }
	 }
}

void Get_balanceStatus(BalanceStatus *blst, RDFCGB_buffer *rdfcgb){
	ADBMS_readCFGB(rdfcgb);
	for(int modIndex = 0; modIndex < NUM_MOD; modIndex++){
		blst[modIndex].balancing_cells = (uint16_t)rdfcgb[modIndex].CFGBR[4] | ((uint16_t)rdfcgb[modIndex].CFGBR[5] << 8);
	}
}
