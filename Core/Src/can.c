/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   CAN1 configuration and BMS-oriented TX/RX helpers.
 *
 * This file provides:
 *  - CubeMX-generated low-level init/deinit for CAN1 and its GPIO/interrupts
 *  - A runtime acceptance filter for StdID 0x604 (charger command) into RX FIFO0
 *  - Thin wrappers to start CAN, enable RX notifications, and queue TX frames
 *  - High-level helpers to serialize BMS data (voltages, temps, summary, safety,
 *    SOC, balance status) into 8-byte payloads and transmit with contiguous IDs
 *
 * Conventions:
 *  - All frames use Standard ID (11-bit) and DLC=8 (data frames).
 *  - Multi-byte integers in payloads are little-endian (LSB first).
 *  - ID ranges/macros (CAN_ID_VOLTAGE, CAN_ID_THERMISTOR, etc.) and array sizes
 *    (NUM_MOD, NUM_CELL_PER_MOD, NUM_THERM_TOTAL) are defined in headers.
 *  - HAL errors are surfaced via return values; Error_Handler() on init failure.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "balance.h"
#include "charger.h"
#include "safety.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sFilterConfig;
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x604 << 5;  // Recieve only ID 0x604
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0xFFF << 5;  // only accept complete match
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	/* USER CODE END CAN1_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (canHandle->Instance == CAN1)
	{
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		/* CAN1 clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PB8     ------> CAN1_RX
		PB9     ------> CAN1_TX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_CAN1_2();

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
		HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{
	if (canHandle->Instance == CAN1)
	{
		/* USER CODE BEGIN CAN1_MspDeInit 0 */
		/* USER CODE END CAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN1 GPIO Configuration
		PB8     ------> CAN1_RX
		PB9     ------> CAN1_TX
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

		/* CAN1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
		/* USER CODE BEGIN CAN1_MspDeInit 1 */
		/* USER CODE END CAN1_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef CAN_start()
{
	return HAL_CAN_Start(&hcan1);
}

HAL_StatusTypeDef CAN_activate()
{
	return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
	{
		return;
	}

	if (rxHeader.IDE == CAN_ID_STD)
	{
		if (rxHeader.StdId == BALANCE_COMMAND_CAN_ID)
		{
			Balance_handleBalanceCANMessage(&rxHeader, rxData);
		}
	}
	else if (rxHeader.IDE == CAN_ID_EXT)
	{
		if (rxHeader.ExtId == ELCON_OUTPUT_CAN_ID)
		{
			Charger_handleElconCANMessage(&rxHeader, rxData);
		}
	}
}

/* ===== TX Path: Queueing a Frame ============================================
 * CAN_Send():
 *  - Waits for a free TX mailbox (or times out after ~10 ms).
 *  - Chooses payload buffer based on StdId range:
 *      Voltage / Thermistor pages / Summary / Safety / SOC / Balance status.
 *  - Adds the message with HAL_CAN_AddTxMessage().
 *
 * Returns HAL status; HAL_TIMEOUT if skip/timeout condition is hit.
 */
HAL_StatusTypeDef CAN_send(CANMessage *ptr, uint8_t length)
{
	if (length > 8)
	{
		length = 8;
	}
	ptr->TxHeader.DLC = length;
	uint32_t previousTime = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
	{
		// printf("waiting\n");
		if (HAL_GetTick() - previousTime > CAN_TIME_OUT_THRESHOLD_MS)
		{
			// printf("timeout\n");
			return HAL_TIMEOUT;
		}
	}
	return HAL_CAN_AddTxMessage(&hcan1, &ptr->TxHeader, ptr->buffer, &ptr->TxMailbox);
}

/* ===== Runtime Settings: Default TX Header ==================================
 * CAN_SettingsInit():
 *  - Starts CAN, enables RX pending notification.
 *  - Sets Standard ID mode, RTR=DATA, DLC=8.
 *  - Caller should update StdId via Set_CAN_Id() per frame.
 */
void CAN_settingsInit(CANMessage *ptr)
{
	CAN_start();
	CAN_activate();
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	ptr->TxHeader.IDE = CAN_ID_STD;
	ptr->TxHeader.StdId = 0x00;
	ptr->TxHeader.RTR = CAN_RTR_DATA;
	ptr->TxHeader.DLC = 0;
}

/** @brief Convenience setter for Standard ID. */
void CAN_setId(CANMessage *ptr, uint32_t id)
{
	ptr->TxHeader.StdId = id;
}

/* ===== High-Level TX: Voltage Pages =========================================
 * CAN_Send_Voltage():
 *  - Sends per-module cell voltages in groups of 4 cells per frame.
 *  - Frame format:
 *      [0..1]=cell j, [2..3]=cell j+1, [4..5]=cell j+2, [6..7]=cell j+3 (LSB..MSB)
 *  - Tail case (if <4 remain): byte[4..7] carry average_volt and sum_volt_module.
 *  - IDs start at CAN_ID_VOLTAGE and increment per frame.
 */
void CAN_sendVoltageData(CANMessage *message, ModuleData *mod)
{
	uint32_t canId = (uint32_t)CAN_ID_VOLTAGE;
	int totalCells = NUM_MOD * NUM_CELL_PER_MOD;

	// Iterate through all cells and pack four cells into each CAN message
	for (int globalCell = 0; globalCell < totalCells; globalCell += 4)
	{
		int byteNumber = 0;

		for (int k = 0; k < 4; k++)
		{
			int cellNumber = globalCell + k;

			if (cellNumber < totalCells)
			{
				int moduleIndex = cellNumber / NUM_CELL_PER_MOD;
				int cellIndex = cellNumber % NUM_CELL_PER_MOD;
				uint16_t voltage_mV = mod[moduleIndex].cellVoltage_mV[cellIndex];

				message->buffer[byteNumber++] = (uint8_t)voltage_mV;
				message->buffer[byteNumber++] = (uint8_t)(voltage_mV >> 8);
			}
			else
			{
				message->buffer[byteNumber++] = 0;
				message->buffer[byteNumber++] = 0;
			}
		}

		CAN_setId(message, canId);
		CAN_send(message, byteNumber);
		canId++;
	}
}

void CAN_sendTemperatureData(CANMessage *message, ModuleData *mod)
{
	uint32_t canId = (uint32_t)CAN_ID_THERMISTOR;
	int totalTherms = NUM_MOD * NUM_THERM_PER_MOD;

	// Iterate through all temps and pack eight temps into each CAN message
	for (int globalTherm = 0; globalTherm < totalTherms; globalTherm += 8)
	{
		int byteNumber = 0;

		for (int k = 0; k < 8; k++)
		{
			int thermNumber = globalTherm + k;

			if (thermNumber < totalTherms)
			{
				int moduleIndex = thermNumber / NUM_THERM_PER_MOD;
				int thermIndex = thermNumber % NUM_THERM_PER_MOD;

				message->buffer[byteNumber++] = (uint8_t)mod[moduleIndex].pointTemp_C[thermIndex];
			}
			else
			{
				message->buffer[byteNumber++] = 0;
			}
		}

		CAN_setId(message, canId);
		CAN_send(message, byteNumber);
		canId++;
	}
}
/* ===== High-Level TX: Cell Summary ==========================================
 * CAN_Send_Cell_Summary():
 *  - Payload:
 *      [0..1] = highest cell voltage (LSB..MSB)
 *      [2..3] = lowest  cell voltage
 *      [4]    = highest cell temperature (8-bit)
 *      [5]    = lowest  cell temperature (8-bit)
 *      [6..7] = (unused)
 */
void CAN_sendPackSummary(CANMessage *message, AccumulatorData *batt)
{
	int byteNumber = 0;
	uint32_t canId = (uint32_t)CAN_ID_PACK_SUMMARY_BASE;
	CAN_setId(message, canId);
	message->buffer[byteNumber++] = (uint8_t)batt->maxCellVoltage_mV;
	message->buffer[byteNumber++] = (uint8_t)(batt->maxCellVoltage_mV >> 8);
	message->buffer[byteNumber++] = (uint8_t)batt->minCellVoltage_mV;
	message->buffer[byteNumber++] = (uint8_t)(batt->minCellVoltage_mV >> 8);
	message->buffer[byteNumber++] = (uint8_t)batt->maxCellTemp_C;
	message->buffer[byteNumber++] = (uint8_t)batt->minCellTemp_C;
	message->buffer[byteNumber++] = (uint8_t)batt->sumPackVoltage_cV;
	message->buffer[byteNumber++] = (uint8_t)(batt->sumPackVoltage_cV >> 8);
	CAN_send(message, byteNumber);

	byteNumber = 0;
	canId++;
	CAN_setId(message, canId);

	FaultMessage_t faultMsg = {0};
	WarningMessage_t warningMsg = {0};
	Safety_getNextFault(&faultMsg);
	Safety_getNextWarning(&warningMsg);

	message->buffer[byteNumber++] = (uint8_t)faultMsg.FaultType;
	message->buffer[byteNumber++] = (uint8_t)warningMsg.WarningType;
	message->buffer[byteNumber++] = (uint8_t)(batt->cellImbalance_mV);
	message->buffer[byteNumber++] = (uint8_t)(batt->cellImbalance_mV >> 8);
	message->buffer[byteNumber++] = (uint8_t)batt->hvSensePackVoltage_cV;
	message->buffer[byteNumber++] = (uint8_t)(batt->hvSensePackVoltage_cV >> 8);
	message->buffer[byteNumber++] = (uint8_t)(batt->soc);
	message->buffer[byteNumber++] = (uint8_t)(batt->soc >> 8);
	CAN_send(message, byteNumber);
}

void CAN_sendModuleSummary(CANMessage *message, ModuleData *mod)
{
	uint32_t canId = (uint32_t)CAN_ID_MODULE_SUMMARY_BASE;
	int byteNumber = 0;
	for (int modIndex = 0; modIndex < NUM_MOD; modIndex++)
	{
		CAN_setId(message, canId);
		message->buffer[byteNumber++] = (uint8_t)mod[modIndex].maxCellVoltage_mV;
		message->buffer[byteNumber++] = (uint8_t)(mod[modIndex].maxCellVoltage_mV >> 8);
		message->buffer[byteNumber++] = (uint8_t)mod[modIndex].minCellVoltage_mV;
		message->buffer[byteNumber++] = (uint8_t)(mod[modIndex].minCellVoltage_mV >> 8);
		message->buffer[byteNumber++] = (uint8_t)mod[modIndex].averageCellVoltage_mV;
		message->buffer[byteNumber++] = (uint8_t)(mod[modIndex].averageCellVoltage_mV >> 8);
		message->buffer[byteNumber++] = (uint8_t)(mod[modIndex].maxCellIndex + 1);
		message->buffer[byteNumber++] = (uint8_t)(mod[modIndex].minCellIndex + 1);

		CAN_send(message, byteNumber);
		canId++;
	}
}

/* ===== High-Level TX: SOC/Current ===========================================
 * CAN_Send_SOC():
 *  - batt->soc is in micro-units (project scale). Convert to milli-units by /1000.
 *  - percent = (milli_soc / max_capacity) * 100 (uint8_t).
 *  - Payload:
 *      [0..1] = SOC (milli-units, LSB..MSB)
 *      [2]    = percent (0..100)
 *      [3..6] = current (uint32_t, LSB..MSB)
 *      [7]    = (unused)
 */
void CAN_Send_SOC(CANMessage *message, AccumulatorData *batt, uint16_t max_capacity)
{
	int byteNumber = 0;
	uint32_t canId = (uint32_t)CAN_ID_SOC;
	uint8_t percent = (uint8_t)((float)(batt->soc / 1000) * 100 / (float)max_capacity); // 1000 for micro->milli
	uint16_t soc = (uint16_t)(batt->soc / 1000);
	CAN_setId(message, canId);
	message->buffer[byteNumber++] = soc & 0xFF;
	message->buffer[byteNumber++] = (soc >> 8) & 0xFF;
	message->buffer[byteNumber++] = (uint8_t)batt->hvSensePackVoltage_cV;
	message->buffer[byteNumber++] = (uint8_t)(batt->hvSensePackVoltage_cV >> 8);
	message->buffer[byteNumber++] = batt->current_mA & 0xFF;
	message->buffer[byteNumber++] = (batt->current_mA >> 8) & 0xFF;
	message->buffer[byteNumber++] = (batt->current_mA >> 16) & 0xFF;
	message->buffer[byteNumber++] = (batt->current_mA >> 24) & 0xFF;
	//    printf("can id for soc: %d\n", CAN_ID);
	CAN_send(message, byteNumber);
}

void CAN_sendBalanceStatus(CANMessage *message, BalanceStatus *blst)
{
	uint32_t canId = (uint32_t)CAN_ID_BALANCE_STATUS;
	for (int moduleIndex = 0; moduleIndex < NUM_MOD;)
	{
		memset(message->buffer, 0, sizeof(message->buffer));
		int byteNumber = 0;
		for (int currentFrameIndex = 0; currentFrameIndex < 4; currentFrameIndex++)
		{
			uint16_t data = (moduleIndex < NUM_MOD) ? blst[moduleIndex].cellsBalancing : 0;
			message->buffer[byteNumber++] = (uint8_t)data;
			message->buffer[byteNumber++] = (uint8_t)(data >> 8);
			moduleIndex++;
		}
		CAN_setId(message, canId);
		CAN_send(message, byteNumber);
		canId++;
	}
	return;
}

void CAN_sendFaultStatus(CANMessage *message)
{
	uint32_t canId = (uint32_t)CAN_ID_Fault_Status;
	uint16_t faultBitStorage[NUM_MOD];

	Safety_getModuleFaultBits(faultBitStorage);

	for (uint8_t start = 0; start < NUM_MOD; start += 4)
	{
		for (uint8_t reset = 0; reset < 8; reset++)
		{
			message->buffer[reset] = 0; // make sure the whole can message starts at 00000000
		}
		int byteNumber = 0;
		for (uint8_t i = start; i < (start + 4) && (i < NUM_MOD); i++)
		{
			uint16_t faultBits = faultBitStorage[i];

			message->buffer[byteNumber++] = (uint8_t)faultBits;
			message->buffer[byteNumber++] = (uint8_t)(faultBits >> 8);
		}
		CAN_setId(message, canId);
		CAN_send(message, byteNumber);
		canId++;
	}
}

void CAN_sendCanHeartBeat(CANMessage *message)
{
	uint32_t canId = (uint32_t)CAN_ID_HeartBeat_Message;
	int byteNumber = 0;
	static uint32_t previousTime = 0;
	uint32_t presentTime = HAL_GetTick();

	if ((presentTime - previousTime) < 1000u)
	{
		return;
	}
	previousTime = presentTime;

	CAN_setId(message, canId);

	message->buffer[byteNumber] = 1;

	CAN_send(message, (uint8_t)byteNumber);
}

void CAN_sendFaultAndWarningSummary(CANMessage *message)
{
	uint32_t canId = (uint32_t)CAN_ID_Fault_And_Warning_Summary;
	FaultMessage_t faultMsg;
	WarningMessage_t warningMsg;
	uint8_t byteNumber = 0;
	bool hasFault;
	bool hasWarning;

	hasFault = Safety_getNextFault(&faultMsg);
	hasWarning = Safety_getNextWarning(&warningMsg);

	CAN_setId(message, canId);

	if (hasFault)
	{
		message->buffer[byteNumber++] = faultMsg.ModuleID;
		message->buffer[byteNumber++] = faultMsg.CellID;
		message->buffer[byteNumber++] = faultMsg.FaultType;
		printf("fault type: %d", faultMsg.FaultType);
	}
	else
	{
		message->buffer[byteNumber++] = 0;
		message->buffer[byteNumber++] = 0;
		message->buffer[byteNumber++] = 0;
	}

	if (hasWarning)
	{
		message->buffer[byteNumber++] = warningMsg.ModuleID;
		message->buffer[byteNumber++] = warningMsg.CellID;
		message->buffer[byteNumber++] = warningMsg.WarningType;
	}
	else
	{
		message->buffer[byteNumber++] = 0;
		message->buffer[byteNumber++] = 0;
		message->buffer[byteNumber++] = 0;
	}

	CAN_send(message, byteNumber);
}
// void CAN_Send_Sensor(struct CANMessage *ptr, batteryModule *batt) {
//     uint16_t CAN_ID = 0x602;
//	Set_CAN_Id(ptr, CAN_ID);
//
//	for (int i = 0; i < NUM_MOD; ++i) {
//		ptr->data[0] = batt->pressure  [i];
//		ptr->data[1] = batt->pressure  [i] >> 8;
//		ptr->data[2] = batt->atmos_temp[i];
//		ptr->data[3] = batt->atmos_temp[i] >> 8;
//		ptr->data[4] = batt->humidity  [i];
//		ptr->data[5] = batt->humidity  [i] >> 8;
//		ptr->data[6] = batt->dew_point [i];
//		ptr->data[7] = batt->dew_point [i] >> 8;
//		CAN_Send(ptr);
//
//		CAN_ID++;
//		Set_CAN_Id(ptr, CAN_ID);
//	}
// }
/* USER CODE END 1 */
