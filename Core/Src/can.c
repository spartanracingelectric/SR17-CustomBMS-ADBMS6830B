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
#include "usart.h"
#include "stdio.h"
#include "balance.h"
#include "safety.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
/* CAN1 init function */
/* ===== CubeMX: CAN1 init ====================================================
 * Bit timing:
 *  - Prescaler=9, BS1=3TQ, BS2=4TQ, SJW=1TQ → 1 + 3 + 4 = 8 TQ per bit
 *  - Effective bitrate depends on APB clock; adjust in .ioc as needed.
 *
 * Filter:
 *  - Bank 0, 32-bit mask, FIFO0
 *  - Accept only StdID 0x604 (exact match) used by the charger command path.
 *
 * Auto features:
 *  - AutoBusOff=ENABLE, AutoWakeUp=DISABLE, AutoRetransmission=ENABLE
 */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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

/* ===== CubeMX: MSP (GPIO/IRQ/Clock) ======================================== */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
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

/* ===== CubeMX: MSP DeInit =================================================== */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */
	  can_skip_flag = 0;
  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

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
/* ===== Runtime State & Thin HAL Wrappers ====================================
 * can_skip_flag: exit condition for a blocked TX wait loop on timeout.
 * CAN_Start():   starts CAN peripheral.
 * CAN_Activate(): enables RX FIFO0 message pending interrupt.
 */
uint8_t can_skip_flag = 0;

HAL_StatusTypeDef CAN_start() { return HAL_CAN_Start(&hcan1); }

HAL_StatusTypeDef CAN_activate() {
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
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
HAL_StatusTypeDef CAN_send(CANMessage *ptr, uint8_t length) {
    if(length > 8) {
    	length = 8;
    }
    ptr->TxHeader.DLC = length;
    uint32_t previousTime = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    	printf("waiting\n");
    	if(HAL_GetTick() - previousTime > CAN_TIME_OUT_THRESHOLD_MS){
			printf("timeout\n");
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
void CAN_settingsInit(CANMessage *ptr) {
    CAN_start();
    CAN_activate();
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    ptr->TxHeader.IDE = CAN_ID_STD;
    ptr->TxHeader.StdId = 0x00;
    ptr->TxHeader.RTR = CAN_RTR_DATA;
    ptr->TxHeader.DLC = 0;
}

/** @brief Convenience setter for Standard ID. */
void CAN_setId(CANMessage *ptr, uint32_t id) { ptr->TxHeader.StdId = id; }

/* ===== High-Level TX: Voltage Pages =========================================
 * CAN_Send_Voltage():
 *  - Sends per-module cell voltages in groups of 4 cells per frame.
 *  - Frame format:
 *      [0..1]=cell j, [2..3]=cell j+1, [4..5]=cell j+2, [6..7]=cell j+3 (LSB..MSB)
 *  - Tail case (if <4 remain): byte[4..7] carry average_volt and sum_volt_module.
 *  - IDs start at CAN_ID_VOLTAGE and increment per frame.
 */
void CAN_sendVoltageData(CANMessage *buffer, ModuleData *mod)
{
	uint32_t canId = (uint32_t)CAN_ID_VOLTAGE;
    for (int i = 0; i < NUM_MOD; i++) {  //pack every 4 cell group in 1 CAN message
    	for (int j = 0; j < NUM_CELL_PER_MOD; j += 4)
    	{
    		int byteNumber = 0;
    		if(j + 3 < NUM_CELL_PER_MOD)
    		{
				buffer->buffer[byteNumber++] =  (uint8_t)mod[i].cell_volt[  j  ];
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].cell_volt[  j  ] >> 8);
				buffer->buffer[byteNumber++] =  (uint8_t)mod[i].cell_volt[j + 1];
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].cell_volt[j + 1] >> 8);
				buffer->buffer[byteNumber++] =  (uint8_t)mod[i].cell_volt[j + 2];
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].cell_volt[j + 2] >> 8);
				buffer->buffer[byteNumber++] =  (uint8_t)mod[i].cell_volt[j + 3];
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].cell_volt[j + 3] >> 8);
		//        printf("can id for voltage: %d\n", CAN_ID);

				CAN_setId(buffer, canId);
				CAN_send(buffer, byteNumber);
				canId++;
    		}

			else
			{
				buffer->buffer[byteNumber++] =  (uint8_t)mod[i].cell_volt[  j  ];
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].cell_volt[  j  ] >> 8);
				buffer->buffer[byteNumber++] =  (uint8_t)mod[i].cell_volt[j + 1];
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].cell_volt[j + 1] >> 8);
				buffer->buffer[byteNumber++] = (uint8_t)mod[i].totalCellVoltage_mV;
				buffer->buffer[byteNumber++] = (uint8_t)(mod[i].totalCellVoltage_mV >> 8);

				CAN_setId(buffer, canId);
				CAN_send(buffer, byteNumber);
				canId++;

			}
    	}
    }
}

/* ===== High-Level TX: Thermistors & Ambient Sensors =========================
 * CAN_Send_Temperature():
 *  - Emits two frames per 12-entry block.
 *  - Frame A: byte[0..7] = temps[i + 0 .. i + 7]
 *  - Frame B: byte[0..3] = temps[i + 8 .. i + 11]
 *             byte[4]    = pressure (LSB)
 *             byte[5]    = atmos_temp (LSB)
 *             byte[6]    = humidity (LSB)
 *             byte[7]    = dew_point (LSB)
 *  - IDs start at CAN_ID_THERMISTOR and increment per frame.
 */
void CAN_sendTemperatureData(CANMessage *buffer, ModuleData *mod)
{
    uint32_t canId = (uint32_t)CAN_ID_THERMISTOR;

    for (int i = 0; i < NUM_MOD; i++)
    {
    	for(int j = 0; j < NUM_THERM_TOTAL; j+= NUM_THERM_PER_MESSAGE)
    	{
    		int byteNumber = 0;

    		if(j + 3 < NUM_THERM_TOTAL)
    		{
    			CAN_setId(buffer, canId);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[  j  ]);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[  j  ] >> 8);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j + 1]);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j + 1] >> 8);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j + 2]);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j + 2] >> 8);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j + 3]);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j + 3] >> 8);

    			CAN_send(buffer, byteNumber);
    			canId++;

    			CAN_setId(buffer, canId);
    		}
    		else
    		{
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[  j  ]);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[  j  ] >> 8);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j +  1]);
    			buffer->buffer[byteNumber++] = (uint8_t)(mod[i].pointTemp_C[j +  1] >> 8);

    			CAN_send(buffer, byteNumber);
    			canId++;

    			CAN_setId(buffer, canId);
    		}
    	}
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
void CAN_sendPackSummary(CANMessage *buffer, AccumulatorData *batt) {
	int byteNumber = 0;
	uint32_t canId = (uint32_t)CAN_ID_SUMMARY;
	CAN_setId(buffer, canId);
	buffer->buffer[byteNumber++] =  (uint8_t)batt->cell_volt_highest;
	buffer->buffer[byteNumber++] = (uint8_t)(batt->cell_volt_highest >> 8);
	buffer->buffer[byteNumber++] =  (uint8_t)batt->cell_volt_lowest;
	buffer->buffer[byteNumber++] = (uint8_t)(batt->cell_volt_lowest >> 8);
	buffer->buffer[byteNumber++] = (uint8_t)batt->cell_temp_highest;
	buffer->buffer[byteNumber++] = (uint8_t)batt->cell_temp_lowest;
	buffer->buffer[byteNumber++] = (uint8_t)batt->total_pack_voltage;
//	printf("can id for summary: %d\n", CAN_ID);
//	ptr->data[6] =
//	ptr->data[7] =
	CAN_send(buffer, byteNumber);
//	printf("Summary\n");
}

void CAN_sendModuleSummary(CANMessage *buffer, ModuleData *mod) {
	uint32_t canId = (uint32_t)CAN_ID_MODULE_SUMMARY_BASE;
	int byteNumber = 0;
	for (int i = 0; i < NUM_MOD; i++) {
		CAN_setId(buffer, canId);
		buffer->buffer[byteNumber++] =  (uint8_t)mod[i].maxCellVoltage_mV;
		buffer->buffer[byteNumber++] = (uint8_t)(mod[i].maxCellVoltage_mV		>> 8);
		buffer->buffer[byteNumber++] =  (uint8_t)mod[i].minCellVoltage_mV;
		buffer->buffer[byteNumber++] = (uint8_t)(mod[i].minCellVoltage_mV  		>> 8);
		buffer->buffer[byteNumber++] =  (uint8_t)mod[i].averageCellVoltage_mV;
		buffer->buffer[byteNumber++] = (uint8_t)(mod[i].averageCellVoltage_mV		>> 8);
		buffer->buffer[byteNumber++] =  (uint8_t)mod[i].maxCellIndex;
		buffer->buffer[byteNumber++] = (uint8_t)(mod[i].minCellIndex  	>> 8);

		CAN_send(buffer,byteNumber);
		canId++;
	}
}

/* ===== High-Level TX: Safety/Health =========================================
 * CAN_Send_Safety_Checker():
 *  - Computes cell_difference = highest - lowest (mV)
 *  - Payload:
 *      [0]    = warnings bitfield
 *      [1]    = faults bitfield
 *      [2..3] = cell_difference (mV)
 *      [4..5] = hvsens_pack_voltage (mV)
 *      [6..7] = sum_pack_voltage (mV)
 */
void CAN_Send_Safety_Checker(CANMessage *buffer, AccumulatorData *batt, uint8_t *faults, uint8_t *warnings) {
	int byteNumber = 0;
	batt->cell_difference = batt->cell_volt_highest - batt->cell_volt_lowest;
	uint32_t canId = (uint32_t)CAN_ID_SAFETY;
	CAN_setId(buffer, canId);
	buffer->buffer[byteNumber++] = *warnings;
	buffer->buffer[byteNumber++] = *faults;
	buffer->buffer[byteNumber++] =  batt->cell_difference           & 0xFF;
	buffer->buffer[byteNumber++] = (batt->cell_difference     >> 8) & 0xFF;
	buffer->buffer[byteNumber++] =  batt->hvsens_pack_voltage       & 0xFF;
	buffer->buffer[byteNumber++] = (batt->hvsens_pack_voltage >> 8) & 0xFF;
	buffer->buffer[byteNumber++] =  batt->sum_pack_voltage          & 0xFF;
	buffer->buffer[byteNumber++] = (batt->sum_pack_voltage    >> 8) & 0xFF;
//	printf("can id for safety: %d\n", CAN_ID);
	CAN_send(buffer, byteNumber);
//	printf("Faults\n");
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
void CAN_Send_SOC(CANMessage *buffer, AccumulatorData *batt, uint16_t max_capacity) {
	int byteNumber = 0;
	uint32_t canId = (uint32_t)CAN_ID_SOC;
    uint8_t percent = (uint8_t)((float) (batt->soc / 1000) * 100 / (float) max_capacity);  // 1000 for micro->milli
    uint16_t soc = (uint16_t) (batt->soc / 1000);
    CAN_setId(buffer, canId);
	buffer->buffer[byteNumber++] = soc & 0xFF;
	buffer->buffer[byteNumber++] = (soc >> 8) & 0xFF;
    buffer->buffer[byteNumber++] = percent;
    buffer->buffer[byteNumber++] = batt->current & 0xFF;
    buffer->buffer[byteNumber++] = (batt->current >> 8) & 0xFF;
    buffer->buffer[byteNumber++] = (batt->current >> 16) & 0xFF;
    buffer->buffer[byteNumber++] = (batt->current >> 24)& 0xFF;
//    printf("can id for soc: %d\n", CAN_ID);
    CAN_send(buffer, byteNumber);
}

/* ===== High-Level TX: Balance DCC Bitmaps ===================================
 * CAN_Send_Balance_Status():
 *  - Two frames; each carries 4x uint16_t bitmaps (LSB..MSB).
 *  - Frame 1 ID = CAN_ID_BALANCE_STATUS  → [0..1]=idx0, [2..3]=idx1, [4..5]=idx2, [6..7]=idx3
 *  - Frame 2 ID = CAN_ID_BALANCE_STATUS+1→ [0..1]=idx4, [2..3]=idx5, [4..5]=idx6, [6..7]=idx7
 */
void CAN_sendBalanceStatus(CANMessage *buffer, BalanceStatus *blst) {
	uint32_t canId = (uint32_t)CAN_ID_BALANCE_STATUS;
	int byteNumber = 0;
	for(int modIndex = 0; modIndex < NUM_MOD; modIndex+=4)
	{
		CAN_setId(buffer, canId);
		if(modIndex >= 8)
		{
			buffer->balanceStatus[byteNumber++] =   (uint8_t)blst[modIndex    ] .cellsBalancing;
			buffer->balanceStatus[byteNumber++] =  (uint8_t)(blst[modIndex    ] .cellsBalancing >> 8);
			buffer->balanceStatus[byteNumber++] =   (uint8_t)blst[modIndex + 1] .cellsBalancing ;
			buffer->balanceStatus[byteNumber++] =  (uint8_t)(blst[modIndex + 1] .cellsBalancing >> 8);
			buffer->balanceStatus[byteNumber++] =   0;
			buffer->balanceStatus[byteNumber++] =   0;
			buffer->balanceStatus[byteNumber++] =   0;
			buffer->balanceStatus[byteNumber++] =   0;
		}
		else
		{
			buffer->balanceStatus[byteNumber++] =   (uint8_t)blst[modIndex    ] .cellsBalancing;
			buffer->balanceStatus[byteNumber++] =  (uint8_t)(blst[modIndex    ] .cellsBalancing >> 8);
			buffer->balanceStatus[byteNumber++] =   (uint8_t)blst[modIndex + 1] .cellsBalancing;
			buffer->balanceStatus[byteNumber++] =  (uint8_t)(blst[modIndex + 1] .cellsBalancing >> 8);
			buffer->balanceStatus[byteNumber++] =   (uint8_t)blst[modIndex + 2] .cellsBalancing;
			buffer->balanceStatus[byteNumber++] =  (uint8_t)(blst[modIndex + 2] .cellsBalancing >> 8);
			buffer->balanceStatus[byteNumber++] =   (uint8_t)blst[modIndex + 3] .cellsBalancing;
			buffer->balanceStatus[byteNumber++] =  (uint8_t)(blst[modIndex + 3] .cellsBalancing >> 8);
		}
//		printf("M1 balancing status %X\n", blst[0].balancing_cells);

		CAN_send(buffer, byteNumber);
		canId++;
	}
	return;
}


void CAN_sendFaultStatus(CANMessage *message)
{
	uint32_t canId = (uint32_t) CAN_ID_Fault_Status;
	uint16_t faultBitStorage[NUM_MOD];

	
		for (uint8_t i = 0; i < NUM_MOD; i++)
		{
			uint16_t faultBits = 0;

			for (uint8_t j = 0; j < NUM_CELL_PER_MOD; j++)
			{
				FaultFlags_t f = GlobalFaults[i][j];
				if (f.UnderVoltage || f.OpenWire || f.PEC || f.OverTemp ||
    			f.UnderTemp || f.OverVoltage || f.RedundancyVolt || f.RedundancyTemp)
				{
    				faultBits |= (uint16_t)(1u << j);
				}
			}
			faultBitStorage[i] = faultBits;
		}
	for (uint8_t start = 0; start < NUM_MOD; start += 4 )
	{
		for (uint8_t j = 0; j < 8; j++)
		{
			message-> buffer[j] = 0; // make sure the whole can message starts at 00000000
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
//void CAN_Send_Sensor(struct CANMessage *ptr, batteryModule *batt) {
//    uint16_t CAN_ID = 0x602;
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
//}
/* USER CODE END 1 */
