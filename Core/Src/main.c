/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accumulator.h"
#include "adbms6830b.h"
#include "balance.h"
#include "charger.h"
#include "contactor_sense.h"
#include "hv_sense.h"
#include "module.h"
#include "safety.h"
#include "soc.h"
#include "shunt.h"
#include "string.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "eeprom.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// static uint8_t BMS_MUX_PAUSE[2][6] = {{0x69, 0x28, 0x0F, 0x09, 0x7F, 0xF9},
//                                       {0x69, 0x08, 0x0F, 0x09, 0x7F, 0xF9}};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	AccumulatorData accmData;
	ModuleData modData[NUM_MOD];
	BalanceStatus balanceStatus[NUM_MOD] = {0};
	ConfigurationRegisterB configB[NUM_MOD] = {0};
	CANMessage msg;

	//	uint8_t moduleCounts = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim1);

	CAN_settingsInit(&msg);

	// Start ADC conversions for HV Sense
	HAL_ADCEx_Calibration_Start(&hadc1);
	ADC1_startConversion();

	ADBMS_csHigh();
	ADBMS_generateCrcTables();
	ADBMS_init();

	Module_init(modData);
	Accumulator_init(&accmData);
	Balance_init(balanceStatus, configB);
	Charger_init(&accmData);
	Shunt_init();
	// TODO: Check if this is needed
	// Sending a fault signal and reseting it
	// HAL_GPIO_WritePin(MCU_SHUTDOWN_SIGNAL_GPIO_Port, MCU_SHUTDOWN_SIGNAL_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	ClearFaultSignal(); // those are for debug the charger and mobo
	SOC_init(&accmData, modData);
	// HAL_ADCEx_Calibration_Start(&hadc2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		uint32_t start = HAL_GetTick();
		Module_getCellVoltages(modData);
		Module_getVoltageStats(modData);
		Accumulator_getVoltageStats(&accmData, modData);
		HVSense_getPackVoltage(&accmData);
		Module_getTemperatures(modData);
		Module_getTemperatureStats(modData);
		Accumulator_getTemperatureStats(&accmData, modData);
		Shunt_updateAccumulator(&accmData);

		// ContactorSense_getContactorState(&accmData);

		// Voltage readings are wrong during balancing so don't check faults while balancing
		if (currentBalanceState != BALANCE_STATE_ACTIVE) {
			Safety_checkFaults(&accmData, modData);
		}

		// Passive balancing is called unless a fault has occurred
		Balance_handleBalancing(modData, &accmData, balanceStatus, configB);

		CAN_sendPackSummary(&msg, &accmData);
		CAN_sendVoltageData(&msg, modData);
		CAN_sendTemperatureData(&msg, modData);
		CAN_Send_SOC(&msg, &accmData, MAX_BATTERY_CAPACITY);
		CAN_sendBalanceStatus(&msg, balanceStatus);
		CAN_sendModuleSummary(&msg, modData);
		CAN_sendFaultStatus(&msg);
		CAN_sendFaultAndWarningSummary(&msg);
		CAN_sendCanHeartBeat(&msg);
		CAN_sendBalanceState(&msg);
		CAN_sendSafetyChecker(&msg, &accmData);

		uint32_t end = HAL_GetTick();
		printf("cycle time ms: %d\n", end - start);
		HAL_GPIO_TogglePin(MCU_HEARTBEAT_LED_GPIO_Port, MCU_HEARTBEAT_LED_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state
	 */
	__disable_irq();
	while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line
	   number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
	   file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
