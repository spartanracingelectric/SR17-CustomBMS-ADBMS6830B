/**
 * @file    hv_sense.c
 */
#include "adc.h"
#include "main.h"
#include "usart.h"
#include <hv_sense.h>
#include <stdio.h>

void HVSense_getPackVoltage(AccumulatorData *batt)
{
	float hvSenseRawVoltage_mV = ADC_getHvSenseRawVoltage_mV();
	float dividerVoltage_mV = hvSenseRawVoltage_mV / GAIN_TLV9001;
	float hvPackVoltage_mV = dividerVoltage_mV * DIVIDER_RATIO;

	if (hvPackVoltage_mV > 5000) // If HV sense is greater than 5V (connected)
	{
		batt->hvSensePackVoltage_cV = hvPackVoltage_mV / 10;
	}
	else
	{
		batt->hvSensePackVoltage_cV = 0;
	}
}
