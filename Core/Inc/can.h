/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   Public interface for CAN1 configuration and BMS CAN helpers.
  *
  * This header collects:
  *  - ID map and constants for BMS CAN payloads (voltages, temps, summary, etc.)
  *  - External state (handles, flags, module arrays) used by the CAN layer
  *  - Prototypes for init/start/notify wrappers and high-level send helpers
  *
  * Conventions:
  *  - All frames use Standard ID (11-bit) and DLC=8 unless noted.
  *  - Multi-byte values in payloads are serialized little-endian (LSB first).
  *  - Array sizes derive from NUM_MOD / NUM_CELLS / NUM_THERM_TOTAL (see main.h).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


typedef struct CANMessage{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t buffer[8];
} CANMessage;


/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
/* ===== CAN ID Map & Serialization ===========================================
 * Base IDs:
 *  - CAN_ID_VOLTAGE:     start ID for voltage frames (groups of four cells).
 *  - CAN_ID_THERMISTOR:  start ID for thermistor/ambient frames (two per block).
 *  - CAN_ID_SUMMARY:     pack-level extremes (voltage/temperature).
 *  - CAN_ID_SAFETY:      warnings/faults, delta, pack voltages.
 *  - CAN_ID_SOC:         state of charge and pack current.
 *  - CAN_ID_BALANCE_STATUS: two frames with DCC bitmaps for 8 modules.
 *
 * Payload sizes:
 *  - CAN_BYTE_NUM:       8 bytes per frame (DLC=8).
 *  - CAN_MESSAGE_NUM_*:  helper counts derived from pack topology.
 *  - *_SIZE:             width helpers for packing 16-bit / 8-bit values.
 */
#define CAN_ID_VOLTAGE 				0x630
#define CAN_ID_THERMISTOR 			0x680
#define CAN_ID_SUMMARY				0x622
#define CAN_ID_SAFETY 				0x600
#define CAN_ID_SOC 					0x621
#define CAN_ID_BALANCE_STATUS		0x623
#define CAN_ID_MODULE_SUMMARY_BASE  0x6A4 // hex of 1700 can id #
#define CAN_BYTE_NUM				8
#define CAN_MESSAGE_NUM_VOLTAGE 	NUM_CELLS * 2 / CAN_BYTE_NUM
#define CAN_MESSAGE_NUM_THERMISTOR 	NUM_THERM_TOTAL / CAN_BYTE_NUM
#define CAN_16BIT_SIZE				2
#define CAN_8BIT_SIZE 				1
#define CAN_TIME_OUT_THRESHOLD_MS	10
#define NUM_THERM_PER_MESSAGE		4
#define CAN_ID_Fault_Status			0x6AE

extern ModuleData modData[NUM_MOD];

extern uint8_t can_skip_flag;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
/* ===== Public API: Runtime Control ==========================================
 * CAN_Start():      Start the CAN peripheral.
 * CAN_Activate():   Enable RX FIFO0 pending-message notifications.
 * CAN_Send():       Queue a frame; auto-selects payload buffer by ID range.
 * CAN_SettingsInit(): Initialize runtime header defaults (IDE/RTR/DLC) and IRQs.
 * Set_CAN_Id():     Convenience setter for Standard ID.
 */
HAL_StatusTypeDef CAN_Start();

HAL_StatusTypeDef CAN_Activate();

HAL_StatusTypeDef CAN_send(CANMessage *ptr, uint8_t length);

void CAN_settingsInit(CANMessage *ptr);

void CAN_setId(CANMessage *ptr, uint32_t id);

/* ===== Public API: High-Level TX Helpers ====================================
 * CAN_Send_Voltage():        voltages in groups of 4 cells per frame.
 * CAN_Send_Temperature():    thermistors (0..11) + ambient sensors per block.
 * CAN_Send_Cell_Summary():   extremes of cell voltage/temperature.
 * CAN_Send_Safety_Checker(): warnings/faults, delta, pack voltages.
 * CAN_Send_SOC():            state-of-charge (milli-units) and current.
 * CAN_Send_Balance_Status(): two frames of per-module DCC bitmaps.
 *
 * Notes:
 *  - All helpers serialize multi-byte fields little-endian into 8-byte payloads.
 *  - Caller should set or rely on contiguous IDs starting at the defined bases.
 */
void CAN_sendVoltageData(CANMessage *ptr, ModuleData *mod);
void CAN_sendTemperatureData(CANMessage *buffer, ModuleData *mod);
void CAN_sendCellSummary(CANMessage *ptr, struct AccumulatorData *batt);
void CAN_sendModuleSummary(CANMessage *ptr, ModuleData *mod);
void CAN_Send_Safety_Checker(CANMessage *ptr, struct AccumulatorData *batt, uint8_t* faults, uint8_t* warnings); // change to camel case
void CAN_Send_SOC(CANMessage *ptr, AccumulatorData *batt, uint16_t max_capacity);
void CAN_sendBalanceStatus(CANMessage *buffer, BalanceStatus *blst);
void CAN_sendFaultStatus(CANMessage *buffer);
//void CAN_Send_Sensor(struct CANMessage *ptr, batteryModule *batt);
/* USER CODE END Prototypes */

//moved can message from main.h to can.h


#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

