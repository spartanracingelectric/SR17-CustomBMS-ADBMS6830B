#include "main.h"
#include <stdio.h>
#include "accumulator.h"
void accumulator_init(AccumulatorData *acc){
	acc->cell_volt_lowest = 0x0000;
	acc->cell_volt_highest = 0x0000;
	acc->cell_difference = 0x0000;
	acc->cell_temp_lowest = 0x0000;
	acc->cell_temp_highest = 0x0000;
	acc->sum_pack_voltage = 0x0000;
	acc->hvsens_pack_voltage = 0x0000;
	acc->balance_status[NUM_MOD] = 0x0000;
	acc->soc = 0x00000000; // microamps!!!!!
	acc->current = 0x00000000;
}

void accumulator_getMinVolatage(AccumulatorData *acc, ModuleData *mod){
	//finding highest and lowest cell voltage
	acc->cell_volt_lowest = mod[0].cell_volt[0];

	for (int modIndex = 0; modIndex < NUM_MOD; modIndex++) {
		for(int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++){
			if (mod[modIndex].cell_volt[cellIndex] < acc->cell_volt_lowest) {
				acc->cell_volt_lowest = mod[modIndex].cell_volt[cellIndex];
			}
		}
	}
}

void accumulator_getMaxVolatage(AccumulatorData *acc, ModuleData *mod){
	//finding highest and lowest cell voltage
	acc->cell_volt_highest = mod[0].cell_volt[0];

	for (int modIndex = 0; modIndex < NUM_MOD; modIndex++) {
		for(int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++){
			if (mod[modIndex].cell_volt[cellIndex] > acc->cell_volt_highest) {
				acc->cell_volt_highest = mod[modIndex].cell_volt[cellIndex];
	//			printf("high voltage fault: %d\n", batt->cell_volt_highest);
			}
		}
	}
}





