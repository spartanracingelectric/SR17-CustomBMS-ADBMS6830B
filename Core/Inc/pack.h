#ifndef INC_PACK_H_
#define INC_PACK_H_

#include "module.h"

typedef struct PackData
{
	int16_t lowestCellVoltage_mV;
	int16_t highestCellVoltage_mV;
	int16_t cellImbalance_mV;
	int16_t lowestCellTemp_C;
	int16_t highestCellTemp_C;
	int32_t totalPackVoltage_mV;
	uint16_t hvSensePackVoltage_mV;
	uint32_t stateOfCharge;
	uint32_t current;
	int16_t packTemp_C;
} PackData;

#define MAX_PACK_CAPACITY (NUM_MODULES_TOTAL * MAX_CELL_CAPACITY)

void Pack_init(PackData *pack);
void Pack_getMinVoltage(PackData *pack, ModuleData module[NUM_MODULES_TOTAL]);
void Pack_getMaxVoltage(PackData *pack, ModuleData module[NUM_MODULES_TOTAL]);
void Pack_getTotalVoltage(PackData *pack, ModuleData module[NUM_MODULES_TOTAL]);

#endif /* INC_PACK_H_ */
