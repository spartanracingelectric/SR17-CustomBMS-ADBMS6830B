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
#include <ADBMS.h>
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

/* ===== CFG Working Images (per device) ======================================
 * config:        Writable image used while balancing (DCC bits modified).
 * defaultConfig: Baseline image restored when balancing ends.
 * NOTE: Each row corresponds to one device in the daisy chain (NUM_MOD).
 */
static uint8_t config[8][6] = { { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 },
								{ 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 },
								{ 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 },
								{ 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 } };

static uint8_t defaultConfig[8][6] = {{ 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 },
									  { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 },
									  { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 },
									  { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 } };

/* ===== Public API: Initialization & Tear-down ================================
 * Balance_init():  Clear flags and DCCs, wake chain, write default CFG.
 * Start_Balance(): Run decision algorithm and push updated CFG (if enabled).
 * End_Balance():   If finish flag set, clear DCCs and restore default CFG.
 */
void Balance_init(uint16_t *balanceStatus){
	balance = 0;
	balance_finish = 0;
	Balance_reset(balanceStatus);
	Wakeup_Sleep();
	LTC_writeCFG(NUM_MOD, defaultConfig);
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
void Start_Balance(uint16_t *read_volt, uint16_t lowest, uint16_t *balanceStatus) {
//	printf("balance enable is %d\n", balance);
	if(balance > 0){
		Discharge_Algo(read_volt, lowest , balanceStatus);
		Wakeup_Sleep();
		LTC_writeCFG(NUM_MOD, config);
	}
	else{
		return;
	}
}

void End_Balance(uint16_t *balanceStatus) {
	if(balance_finish == 1){
		Balance_reset(balanceStatus);
		Wakeup_Sleep();
		LTC_writeCFG(NUM_MOD, defaultConfig);
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
void Discharge_Algo(uint16_t *read_volt, uint16_t lowest, uint16_t *balanceStatus) {
	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
		// check if each cell is close within 0.005V of the lowest cell.
		uint8_t DCC[12];
		for (uint8_t cell_idx = 0; cell_idx < NUM_CELL_PER_MOD; cell_idx++) {
			if (read_volt[dev_idx * NUM_CELL_PER_MOD + cell_idx] - lowest > BALANCE_THRESHOLD) {
				DCC[cell_idx] = 1;
				balanceStatus[dev_idx] |= (1 << cell_idx);
			} else {
				DCC[cell_idx] = 0;
				balanceStatus[dev_idx] &= ~(1 << cell_idx); //set the bit to 0
			}
		}
		Set_Cfg(dev_idx, (uint8_t*) DCC);
	}
}

/* ===== Helpers: Clear DCC / Apply CFG =======================================
 * Balance_reset(): Clear all status bits and set DCC=0 for every cell/device.
 * Set_Cfg():       Map DCC array into config[dev][4] (cells 0–7) and
 *                  config[dev][5] (cells 8–15) bit positions.
 */
void Balance_reset(uint16_t *balanceStatus) {
	uint8_t DCC[12] = {0};  //reset all DCC to 0
	for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
		balanceStatus[dev_idx] = 0;
//		printf("balanceStaus[%d]: %d\n", dev_idx, balanceStatus[dev_idx]);

		Set_Cfg(dev_idx, (uint8_t*) DCC);
	}
}

/**
 * @brief  Apply DCC bits for a device to its CFG image.
 *
 * Mapping:
 *  - cell 0..7  → config[dev_idx][4] bit 0..7
 *  - cell 8..15 → config[dev_idx][5] bit 0..7
 *
 * @param dev_idx  Daisy-chain device index
 * @param DCC      Discharge control bits per cell (1=on, 0=off)
 */
void Set_Cfg(uint8_t dev_idx, uint8_t *DCC) {
	for (uint8_t cell_idx = 0; cell_idx < NUM_CELL_PER_MOD; cell_idx++) {
		if (DCC[cell_idx]) {
			if (cell_idx < 8) {
				config[dev_idx][4] |= (1 << cell_idx);
			} else if (cell_idx >= 8) {
				config[dev_idx][5] |= (1 << (cell_idx - 8));
			}
		} else {
			if (cell_idx < 8) {
				config[dev_idx][4] &= (~(1 << cell_idx));
			} else {
				config[dev_idx][5] &= (~(1 << (cell_idx - 8)));
			}
		}
	}
}

