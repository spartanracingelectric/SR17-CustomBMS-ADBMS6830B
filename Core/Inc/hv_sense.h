#ifndef INC_HV_SENSE_H_
#define INC_HV_SENSE_H_

#include "main.h"

#define GAIN_TLV9001 1.58f 		//TLV9001 gain
//Resistor values for the voltage divider
#define R1 1400000.0f  			//1.4 MΩ
#define R2 6761.5f     			//6863kΩ is the spec, but it might be parallel to other components. measured value was 6660kΩ, but it was too low, so I took average of them and it works well.
#define DIVIDER_RATIO (R1 + R2) / R2

void ReadHVInput(batteryModule *batt);

void getSumPackVoltage(batteryModule *batt);

#endif /* INC_HV_SENSE_H_ */
