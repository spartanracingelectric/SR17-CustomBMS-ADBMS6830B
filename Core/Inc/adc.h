/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */
#define NUM_ADC1_CHANNEL 2
#define HV_SENSE_ADC_RANK 2
#define VREFINT_ADC_RANK 1
#define ADC_RESOLUTION 4095.0f

// Find calibration value by reading raw ADC value of VREFINT when powering board (3.3V)
#define VREFINT_CALIBRATION_ADC_RAW_COUNT 1499.0f
#define VREFINT_CALIBRATION_SUPPLY_POWER_MV 3300.0f

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
void ADC1_startConversion(void);
float ADC_getHvSenseRawVoltage_mV(void);
float ADC_getVref_mV(void);
float ADC_convertAdcCountToVoltage_mV(uint16_t adcCount, float vRef_mV);
float ADC_calculateVref_mV(uint16_t vRefAdcCount);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

