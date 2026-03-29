#ifndef INC_ACCUMULATOR_H_
#define INC_ACCUMULATOR_H_

#include "main.h"
#include "module.h"

typedef struct AccumulatorData {
	int16_t maxCellVoltage_mV;
	int16_t minCellVoltage_mV;
	int16_t averageCellVoltage_mV;
	int16_t cellImbalance_mV;
	int16_t averagePointTemp_C;
	int16_t minCellTemp_C;
	int16_t maxCellTemp_C;
	uint16_t sumPackVoltage_cV;
	uint16_t hvSensePackVoltage_cV;
    uint32_t soc; 
    int32_t shuntCurrent_mA;
    int64_t shuntCoulombCount;
    uint8_t contactorState;
    uint16_t atmos_temp;
	uint16_t pressure;
} AccumulatorData;


void Accumulator_init(AccumulatorData *acc);
void Accumulator_getMinVoltage(AccumulatorData *acc, ModuleData *mod);
void Accumulator_getMaxVoltage(AccumulatorData *acc, ModuleData *mod);
void Accumulator_getVoltageStats(AccumulatorData *acc, ModuleData *mod);
void Accumulator_getTotalVoltage(AccumulatorData *batt, ModuleData *mod);
void Accumulator_getTemperatureStats(AccumulatorData *acc, ModuleData *mod);

#endif /* INC_ACCUMULATOR_H_ */
