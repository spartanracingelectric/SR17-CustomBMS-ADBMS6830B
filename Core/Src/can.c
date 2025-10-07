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

HAL_StatusTypeDef CAN_Start() { return HAL_CAN_Start(&hcan1); }

HAL_StatusTypeDef CAN_Activate() {
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
HAL_StatusTypeDef CAN_Send(CANMessage *ptr) {
    uint32_t can_erraps_time = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    	if(can_skip_flag == 1){
    		return HAL_TIMEOUT;
    	}
    	else if(HAL_GetTick() - can_erraps_time > 10){
    		can_skip_flag = 1;
    		return HAL_TIMEOUT;
    	}
    }
    uint8_t *dataPtr = NULL;

	 if(ptr->TxHeader.StdId >= CAN_ID_VOLTAGE &&  ptr->TxHeader.StdId < CAN_ID_VOLTAGE + (NUM_CELLS * 2 / CAN_BYTE_NUM)) {//(NUM_CELLS * 2 / CAN_BYTE_NUM is just a number of can message
	   dataPtr = (uint8_t *)ptr->voltageBuffer;
	 }
	 else if(ptr->TxHeader.StdId >= CAN_ID_THERMISTOR &&  ptr->TxHeader.StdId < CAN_ID_THERMISTOR + ((NUM_THERM_TOTAL + (4 * NUM_MOD)) / CAN_BYTE_NUM)) {//(NUM_THERM_TOTAL + (4 * NUM_MOD)) is a total num of thermistor + sensor, (4 * NUM_MOD) is number of sensors
	   dataPtr = (uint8_t *)ptr->thermistorBuffer;
	 }
	 else if (ptr->TxHeader.StdId == CAN_ID_SUMMARY) {
			dataPtr = (uint8_t *)ptr->summaryBuffer;
	 }
	 else if (ptr->TxHeader.StdId == CAN_ID_SAFETY) {
		dataPtr = (uint8_t *)ptr->safetyBuffer;
	 }
	 else if (ptr->TxHeader.StdId == CAN_ID_SOC) {
		dataPtr = (uint8_t *)ptr->socBuffer;
	 }
	 else if (ptr->TxHeader.StdId == CAN_ID_BALANCE_STATUS || ptr->TxHeader.StdId == CAN_ID_BALANCE_STATUS + 1) {
		dataPtr = (uint8_t *)ptr->balanceStatus;
	}
	return HAL_CAN_AddTxMessage(&hcan1, &ptr->TxHeader, dataPtr, &ptr->TxMailbox);
}

/* ===== Runtime Settings: Default TX Header ==================================
 * CAN_SettingsInit():
 *  - Starts CAN, enables RX pending notification.
 *  - Sets Standard ID mode, RTR=DATA, DLC=8.
 *  - Caller should update StdId via Set_CAN_Id() per frame.
 */
void CAN_SettingsInit(CANMessage *ptr) {
    CAN_Start();
    CAN_Activate();
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    ptr->TxHeader.IDE = CAN_ID_STD;
    ptr->TxHeader.StdId = 0x00;
    ptr->TxHeader.RTR = CAN_RTR_DATA;
    ptr->TxHeader.DLC = 8;
}

/** @brief Convenience setter for Standard ID. */
void Set_CAN_Id(CANMessage *ptr, uint32_t id) { ptr->TxHeader.StdId = id; }

/* ===== High-Level TX: Voltage Pages =========================================
 * CAN_Send_Voltage():
 *  - Sends per-module cell voltages in groups of 4 cells per frame.
 *  - Frame format:
 *      [0..1]=cell j, [2..3]=cell j+1, [4..5]=cell j+2, [6..7]=cell j+3 (LSB..MSB)
 *  - Tail case (if <4 remain): byte[4..7] carry average_volt and sum_volt_module.
 *  - IDs start at CAN_ID_VOLTAGE and increment per frame.
 */
void CAN_Send_Voltage(CANMessage *buffer, ModuleData *mod) {
	uint32_t CAN_ID = (uint32_t)CAN_ID_VOLTAGE;
    for (int i = 0; i < NUM_MOD; i ++) {  //pack every 4 cell group in 1 CAN message
    	for (int j = 0; j < NUM_CELL_PER_MOD; j += 4) {
    		if(j + 3 < NUM_CELL_PER_MOD){
				buffer->voltageBuffer[0] =  mod[i].cell_volt[  j  ]       & 0xFF; 			//To ensure the data type is uint8_t, use & 0xFF
				buffer->voltageBuffer[1] = (mod[i].cell_volt[  j  ] >> 8) & 0xFF;
				buffer->voltageBuffer[2] =  mod[i].cell_volt[j + 1]       & 0xFF;
				buffer->voltageBuffer[3] = (mod[i].cell_volt[j + 1] >> 8) & 0xFF;
				buffer->voltageBuffer[4] =  mod[i].cell_volt[j + 2]       & 0xFF;
				buffer->voltageBuffer[5] = (mod[i].cell_volt[j + 2] >> 8) & 0xFF;
				buffer->voltageBuffer[6] =  mod[i].cell_volt[j + 3]       & 0xFF;
				buffer->voltageBuffer[7] = (mod[i].cell_volt[j + 3] >> 8) & 0xFF;
		//        printf("can id for voltage: %d\n", CAN_ID);

				Set_CAN_Id(buffer, CAN_ID);
				CAN_Send(buffer);
				CAN_ID++;
    		}

			else{
				buffer->voltageBuffer[0] =  mod[i].cell_volt[  j  ]       & 0xFF; 			//To ensure the data type is uint8_t, use & 0xFF
				buffer->voltageBuffer[1] = (mod[i].cell_volt[  j  ] >> 8) & 0xFF;
				buffer->voltageBuffer[2] =  mod[i].cell_volt[j + 1]       & 0xFF;
				buffer->voltageBuffer[3] = (mod[i].cell_volt[j + 1] >> 8) & 0xFF;
				buffer->voltageBuffer[4] =  mod[i].average_volt           & 0xFF;
				buffer->voltageBuffer[5] = (mod[i].average_volt     >> 8) & 0xFF;
				buffer->voltageBuffer[6] =  mod[i].sum_volt_module        & 0xFF;
				buffer->voltageBuffer[7] = (mod[i].sum_volt_module  >> 8) & 0xFF;
		//        printf("can id for voltage: %d\n", CAN_ID);

				Set_CAN_Id(buffer, CAN_ID);
				CAN_Send(buffer);
				CAN_ID++;
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
void CAN_Send_Temperature(CANMessage *buffer, ModuleData *mod) {
    uint32_t CAN_ID = (uint32_t)CAN_ID_THERMISTOR;

    for (int i = 0; i < NUM_THERM_TOTAL; i += 12) {
        Set_CAN_Id(buffer, CAN_ID);
        buffer->thermistorBuffer[0] = (uint8_t)(mod[i].cell_temp[  i  ] & 0xFF);
		buffer->thermistorBuffer[1] = (uint8_t)(mod[i].cell_temp[i + 1] & 0xFF);
		buffer->thermistorBuffer[2] = (uint8_t)(mod[i].cell_temp[i + 2] & 0xFF);
		buffer->thermistorBuffer[3] = (uint8_t)(mod[i].cell_temp[i + 3] & 0xFF);
		buffer->thermistorBuffer[4] = (uint8_t)(mod[i].cell_temp[i + 4] & 0xFF);
		buffer->thermistorBuffer[5] = (uint8_t)(mod[i].cell_temp[i + 5] & 0xFF);
		buffer->thermistorBuffer[6] = (uint8_t)(mod[i].cell_temp[i + 6] & 0xFF);
		buffer->thermistorBuffer[7] = (uint8_t)(mod[i].cell_temp[i + 7] & 0xFF);

//		printf("temp1 in 8 bits:%d\n", ptr->data[0]);
//		printf("temp2 in 8 bits:%d\n", ptr->data[1]);
//		printf("temp3 in 8 bits:%d\n", ptr->data[2]);
//		printf("temp4 in 8 bits:%d\n", ptr->data[3]);
//		printf("temp5 in 8 bits:%d\n", ptr->data[4]);
//		printf("temp6 in 8 bits:%d\n", ptr->data[5]);
//		printf("temp7 in 8 bits:%d\n", ptr->data[6]);
//		printf("temp8 in 8 bits:%d\n", ptr->data[7]);
//        printf("can id for temp1: %d\n", CAN_ID);
		CAN_Send(buffer);
		CAN_ID++;

		Set_CAN_Id(buffer, CAN_ID);

		buffer->thermistorBuffer[0] = (uint8_t)(mod[i].cell_temp [i +  8] & 0xFF);
		buffer->thermistorBuffer[1] = (uint8_t)(mod[i].cell_temp [i +  9] & 0xFF);
		buffer->thermistorBuffer[2] = (uint8_t)(mod[i].cell_temp [i + 10] & 0xFF);
		buffer->thermistorBuffer[3] = (uint8_t)(mod[i].cell_temp [i + 11] & 0xFF);
		buffer->thermistorBuffer[4] = (uint8_t)(mod[i].pressure           & 0xFF);
		buffer->thermistorBuffer[5] = (uint8_t)(mod[i].atmos_temp         & 0xFF);
		buffer->thermistorBuffer[6] = (uint8_t)(mod[i].humidity           & 0xFF);
		buffer->thermistorBuffer[7] = (uint8_t)(mod[i].dew_point          & 0xFF);

//		printf("temp9 in 8 bits:%d\n", ptr->data[0]);
//		printf("temp10 in 8 bits:%d\n", ptr->data[1]);
//		printf("temp11 in 8 bits:%d\n", ptr->data[2]);
//		printf("temp12 in 8 bits:%d\n", ptr->data[3]);
//		printf("can id for temp2: %d\n", CAN_ID);
		CAN_Send(buffer);
		CAN_ID++;
//      printf("sending CAN");
		}
//	for(int i = 0; i < 96; i ++){
//		uint8_t eightbit = (uint8_t)(read_temp[i]);
//		uint16_t sixteenbit = read_temp[i];
//		printf("temp[%d] in 8 bits:%d\n", i, eightbit);
//		printf("temp[%d] in 16 bits:%d\n", i, sixteenbit);
//	}
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
void CAN_Send_Cell_Summary(CANMessage *buffer, AccumulatorData *batt) {
	uint32_t CAN_ID = (uint32_t)CAN_ID_SUMMARY;
	Set_CAN_Id(buffer, CAN_ID);
	buffer->summaryBuffer[0] =  batt->cell_volt_highest         & 0xFF;
	buffer->summaryBuffer[1] = (batt->cell_volt_highest >> 8)   & 0xFF;
	buffer->summaryBuffer[2] =  batt->cell_volt_lowest          & 0xFF;
	buffer->summaryBuffer[3] = (batt->cell_volt_lowest >> 8)    & 0xFF;
	buffer->summaryBuffer[4] = (uint8_t)batt->cell_temp_highest & 0xFF;
	buffer->summaryBuffer[5] = (uint8_t)batt->cell_temp_lowest  & 0xFF;
//	printf("can id for summary: %d\n", CAN_ID);
//	ptr->data[6] =
//	ptr->data[7] =
	CAN_Send(buffer);
//	printf("Summary\n");
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
	batt->cell_difference = batt->cell_volt_highest - batt->cell_volt_lowest;
	uint32_t CAN_ID = (uint32_t)CAN_ID_SAFETY;
	Set_CAN_Id(buffer, CAN_ID);
	buffer->safetyBuffer[0] = *warnings;
	buffer->safetyBuffer[1] = *faults;
	buffer->safetyBuffer[2] =  batt->cell_difference           & 0xFF;
	buffer->safetyBuffer[3] = (batt->cell_difference     >> 8) & 0xFF;
	buffer->safetyBuffer[4] =  batt->hvsens_pack_voltage       & 0xFF;
	buffer->safetyBuffer[5] = (batt->hvsens_pack_voltage >> 8) & 0xFF;
	buffer->safetyBuffer[6] =  batt->sum_pack_voltage          & 0xFF;
	buffer->safetyBuffer[7] = (batt->sum_pack_voltage    >> 8) & 0xFF;
//	printf("can id for safety: %d\n", CAN_ID);
	CAN_Send(buffer);
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
void CAN_Send_SOC(CANMessage *buffer, AccumulatorData *batt,
                  uint16_t max_capacity) {
	uint32_t CAN_ID = (uint32_t)CAN_ID_SOC;
    uint8_t percent = (uint8_t)((float) (batt->soc / 1000) * 100 / (float) max_capacity);  // 1000 for micro->milli
    uint16_t soc = (uint16_t) (batt->soc / 1000);
    Set_CAN_Id(buffer, CAN_ID);
	buffer->socBuffer[0] = soc & 0xFF;
	buffer->socBuffer[1] = (soc >> 8) & 0xFF;
    buffer->socBuffer[2] = percent;
    buffer->socBuffer[3] = batt->current & 0xFF;
    buffer->socBuffer[4] = (batt->current >> 8) & 0xFF;
    buffer->socBuffer[5] = (batt->current >> 16) & 0xFF;
    buffer->socBuffer[6] = (batt->current >> 24)& 0xFF;
//    printf("can id for soc: %d\n", CAN_ID);
    CAN_Send(buffer);
}

/* ===== High-Level TX: Balance DCC Bitmaps ===================================
 * CAN_Send_Balance_Status():
 *  - Two frames; each carries 4x uint16_t bitmaps (LSB..MSB).
 *  - Frame 1 ID = CAN_ID_BALANCE_STATUS  → [0..1]=idx0, [2..3]=idx1, [4..5]=idx2, [6..7]=idx3
 *  - Frame 2 ID = CAN_ID_BALANCE_STATUS+1→ [0..1]=idx4, [2..3]=idx5, [4..5]=idx6, [6..7]=idx7
 */
void CAN_Send_Balance_Status(CANMessage *buffer, BalanceStatus *blst, RDFCGB_buffer *rdfcgb){
	Get_balanceStatus(blst, rdfcgb);
	uint32_t CAN_ID = (uint32_t)CAN_ID_BALANCE_STATUS;
	uint8_t numByte = 8;

	for(int modIndex = 0; modIndex < NUM_MOD; modIndex+=4){
		Set_CAN_Id(buffer, CAN_ID);
		for (int reset = 0; reset < numByte; reset++){ //reset the buffer
			buffer->balanceStatus[reset] = 0;
		}
		if(modIndex >= 8){
			buffer->balanceStatus[0] =   blst[modIndex    ] .balancing_cells        & 0xFF;
			buffer->balanceStatus[1] =  (blst[modIndex    ] .balancing_cells >> 8)  & 0xFF;
			buffer->balanceStatus[2] =   blst[modIndex + 1] .balancing_cells        & 0xFF;
			buffer->balanceStatus[3] =  (blst[modIndex + 1] .balancing_cells >> 8)  & 0xFF;
		}
		else{
			buffer->balanceStatus[0] =   blst[modIndex    ] .balancing_cells        & 0xFF;
			buffer->balanceStatus[1] =  (blst[modIndex    ] .balancing_cells >> 8)  & 0xFF;
			buffer->balanceStatus[2] =   blst[modIndex + 1] .balancing_cells        & 0xFF;
			buffer->balanceStatus[3] =  (blst[modIndex + 1] .balancing_cells >> 8)  & 0xFF;
			buffer->balanceStatus[4] =   blst[modIndex + 2] .balancing_cells        & 0xFF;
			buffer->balanceStatus[5] =  (blst[modIndex + 2] .balancing_cells >> 8)  & 0xFF;
			buffer->balanceStatus[6] =   blst[modIndex + 3] .balancing_cells        & 0xFF;
			buffer->balanceStatus[7] =  (blst[modIndex + 3] .balancing_cells >> 8)  & 0xFF;
		}
		printf("M1 balancing status %d\n", blst[0].balancing_cells);

		CAN_Send(buffer);
		CAN_ID++;
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
