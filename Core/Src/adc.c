/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "adc.h"
#include "stdio.h"
#include "stm32f1xx_hal_dma.h"

/* USER CODE BEGIN 0 */
uint16_t adc1Buffer[NUM_ADC1_CHANNEL] = {0};
bool isAdc1ConversionsFinished = false;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (adcHandle->Instance == ADC1)
	{
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* ADC1 clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**ADC1 GPIO Configuration
		PC5     ------> ADC1_IN15
		*/
		GPIO_InitStruct.Pin = HV_SENSE_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(HV_SENSE_GPIO_Port, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		/* ADC1 Init */
		hdma_adc1.Instance = DMA1_Channel1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc1.Init.Mode = DMA_NORMAL;
		hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

		/* USER CODE BEGIN ADC1_MspInit 1 */

		/* USER CODE END ADC1_MspInit 1 */
	}
	else if (adcHandle->Instance == ADC2)
	{
		/* USER CODE BEGIN ADC2_MspInit 0 */

		/* USER CODE END ADC2_MspInit 0 */
		/* ADC2 clock enable */
		__HAL_RCC_ADC2_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**ADC2 GPIO Configuration
		PC0     ------> ADC2_IN10
		PC1     ------> ADC2_IN11
		PC2     ------> ADC2_IN12
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | Shunt_PIN_FOR_2A_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* USER CODE BEGIN ADC2_MspInit 1 */

		/* USER CODE END ADC2_MspInit 1 */
	}
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
	if (adcHandle->Instance == ADC1)
	{
		/* USER CODE BEGIN ADC1_MspDeInit 0 */

		/* USER CODE END ADC1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		PC5     ------> ADC1_IN15
		*/
		HAL_GPIO_DeInit(HV_SENSE_GPIO_Port, HV_SENSE_Pin);

		/* ADC1 DMA DeInit */
		HAL_DMA_DeInit(adcHandle->DMA_Handle);
		/* USER CODE BEGIN ADC1_MspDeInit 1 */

		/* USER CODE END ADC1_MspDeInit 1 */
	}
	else if (adcHandle->Instance == ADC2)
	{
		/* USER CODE BEGIN ADC2_MspDeInit 0 */

		/* USER CODE END ADC2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC2_CLK_DISABLE();

		/**ADC2 GPIO Configuration
		PC0     ------> ADC2_IN10
		PC1     ------> ADC2_IN11
		PC2     ------> ADC2_IN12
		*/
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | Shunt_PIN_FOR_2A_Pin);

		/* USER CODE BEGIN ADC2_MspDeInit 1 */

		/* USER CODE END ADC2_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		isAdc1ConversionsFinished = true;
	}
}

void ADC1_startConversion(void)
{
	isAdc1ConversionsFinished = false;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1Buffer, NUM_ADC1_CHANNEL);
}

float ADC_getHvSenseRawVoltage_mV(void)
{
	uint32_t startTick = HAL_GetTick();

	while (!isAdc1ConversionsFinished)
	{
		if ((HAL_GetTick() - startTick) >= 10)
		{
			// Timeout after 10 ms
			return 0;
		}
	}

	uint16_t hvSenseAdcValue = adc1Buffer[HV_SENSE_ADC_RANK - 1];
	uint16_t vRefRaw = adc1Buffer[VREFINT_ADC_RANK - 1];

	float vRef_mV = ADC_calculateVref_mV(vRefRaw);

	float hvSenseRawVoltage_mV = ADC_convertAdcCountToVoltage_mV(hvSenseAdcValue, vRef_mV);

	ADC1_startConversion();
	return hvSenseRawVoltage_mV;
}

float ADC_getVref_mV(void)
{
	uint32_t startTick = HAL_GetTick();

	while (!isAdc1ConversionsFinished)
	{
		if ((HAL_GetTick() - startTick) >= 10)
		{
			// Timeout after 10 ms
			return 0;
		}
	}

	uint16_t vRefAdcCount = adc1Buffer[VREFINT_ADC_RANK - 1];
	float vRef_mV = ADC_calculateVref_mV(vRefAdcCount);

	ADC1_startConversion();
	return vRef_mV;
}

float ADC_calculateVref_mV(uint16_t vRefAdcCount)
{
	float vRef_mV = VREFINT_CALIBRATION_SUPPLY_POWER_MV * VREFINT_CALIBRATION_ADC_RAW_COUNT / (float)vRefAdcCount;
	return vRef_mV;
}

float ADC_convertAdcCountToVoltage_mV(uint16_t adcCount, float vRef_mV)
{
	float voltage_mv = ((float)adcCount / ADC_RESOLUTION) * vRef_mV;
	return voltage_mv;
}

/* USER CODE END 1 */
