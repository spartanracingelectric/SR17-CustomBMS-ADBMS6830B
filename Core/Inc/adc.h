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
#define HV_SENSE_ADC_RANK 1
#define VREFINT_ADC_RANK 2
#define VREFINT_CAL 1.2f

#define VREFINT_CALIBRATION_SUPPLY_POWER_MV 3300

// Find calibration value by reading raw ADC value of VREFINT when powering board (3.3V)
#define VREFINT_CALIBRATION_ADC_RAW_COUNT 1200
#define ADC_RESOLUTION 4095.0f
#define V_REF 3.255383f //got this value from vref function(commented out)



/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
uint32_t readADCChannel(uint32_t channel);
float getVref();

void ADC1_startConversion(void);
uint16_t ADC_getHvSenseRawVoltage(void);
uint16_t ADC_getVref(void);
float ADC_convertAdcValueToVoltage(uint16_t adcValue, uint16_t vRef_mV);
uint16_t ADC_calculateVref(uint16_t vRefRaw);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

