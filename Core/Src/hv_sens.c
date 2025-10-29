/**
 * @file    hv_sense.c
 * @brief   High-voltage pack sensing (ADC scaling) and pack-sum aggregation.
 *
 * This file provides:
 *  - ReadHVInput(): sample the HV sense path, undo op-amp/divider scaling, and
 *                   publish the pack voltage into AccumulatorData.
 *  - getSumPackVoltage(): sum per-cell voltages into a pack total.
 *
 * Conventions:
 *  - ADC raw → volts at MCU pin: (adc / ADC_RESOLUTION) * Vref.
 *  - Undo analog front-end: divide by GAIN_TLV9001 (op-amp), then multiply by
 *    DIVIDER_RATIO ((R1+R2)/R2) to reconstruct the pack voltage.
 *  - Stored units: centivolts (cV). I.e., volts * 100 → uint16_t fields.
 *  - HV presence gate: if reconstructed Vpack ≤ 10 V, publish 0 (disconnected).
 *
 * Assumptions:
 *  - ADC_RESOLUTION, GAIN_TLV9001, DIVIDER_RATIO are defined in headers.
 *  - batt->hvsens_pack_voltage and batt->sum_pack_voltage are uint16_t in cV.
 */
#include <hv_sense.h>
#include "adc.h"
#include "main.h"
#include <stdio.h>
#include "usart.h"

/* ===== High-Voltage Sense Path ==============================================
 * ReadHVInput():
 *  - Reads ADC channel tied to the HV sense output.
 *  - Scales raw counts to volts at the MCU pin using the runtime Vref.
 *  - Unscales analog front-end (op-amp + divider) to reconstruct pack voltage.
 *  - Publishes centivolts into batt->hvsens_pack_voltage when > 10 V; else 0.
 */
void ReadHVInput(AccumulatorData *batt) {
	uint32_t adcValue = 0;
	float vRef = 0;

	adcValue = readADCChannel(ADC_CHANNEL_15);
	vRef = getVref();
//		printf("adcValue:%d\n", adcValue);

	//calculate voltage based on  resolution and gain on opamp, voltage divider ratio
	float adcVoltage = ((float)adcValue / ADC_RESOLUTION) * vRef;
//		printf("adcVoltage for hv is: %f\n", adcVoltage);
	float amcOutput = adcVoltage / GAIN_TLV9001;
	float hvInput = (amcOutput) * (DIVIDER_RATIO);
	if(hvInput > 10){//if hvsens is greater than 10V(connected)
		batt->hvsens_pack_voltage = hvInput * 100;
	}
	else{
		batt->hvsens_pack_voltage = 0;
	}
}



