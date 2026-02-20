#include "pack.h"

void Pack_getMinVoltage(PackData *pack, ModuleData *module)
{
	pack->lowestCellVoltage_mV = module[0].cellVoltage_mV[0];

	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] < pack->lowestCellVoltage_mV)
			{
				pack->lowestCellVoltage_mV = module[moduleIndex].cellVoltage_mV[cellIndex];
			}
		}
	}
}

void Pack_getMaxVoltage(PackData *pack, ModuleData *module)
{
	pack->highestCellVoltage_mV = module[0].cellVoltage_mV[0];

	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			if (module[moduleIndex].cellVoltage_mV[cellIndex] > pack->highestCellVoltage_mV)
			{
				pack->highestCellVoltage_mV = module[moduleIndex].cellVoltage_mV[cellIndex];
			}
		}
	}
}

void Pack_getTotalVoltage(PackData *pack, ModuleData *module)
{
	int32_t totalPackVoltage = 0;

	for (int moduleIndex = 0; moduleIndex < NUM_MODULES_TOTAL; moduleIndex++)
	{
		for (int cellIndex = 0; cellIndex < NUM_CELLS_PER_MODULE; cellIndex++)
		{
			totalPackVoltage += module[moduleIndex].cellVoltage_mV[cellIndex];
		}
	}

	pack->totalPackVoltage_mV = totalPackVoltage;
}
