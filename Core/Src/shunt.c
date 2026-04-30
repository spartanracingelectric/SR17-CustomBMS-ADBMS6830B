#include "shunt.h"

static int32_t parseInt32LE(uint8_t *data);
static int64_t parseInt64LE(uint8_t *data);

static Shunt shunt;

void Shunt_init(void){
	shunt.current_mA = 0;
	shunt.coulombCount = 0;
}

void Shunt_handleCANMessage_Current(uint8_t rxData[8]){
	shunt.current_mA = parseInt32LE(rxData);
}

void Shunt_handleCANMessage_Coulomb(uint8_t rxData[8]){
	shunt.coulombCount = parseInt64LE(rxData);
}

void Shunt_updateAccumulator(AccumulatorData* acc){
	acc->shuntCurrent_mA = shunt.current_mA;
	acc->shuntCoulombCount = shunt.coulombCount;
}

static int32_t parseInt32LE(uint8_t *data)
{
    return (int32_t)(
        ((int32_t)data[0] << 0) |
        ((int32_t)data[1] << 8) |
        ((int32_t)data[2] << 16)  |
        ((int32_t)data[3] << 24)
    );
}

static int64_t parseInt64LE(uint8_t *data){
	return (int64_t)(
		((int64_t)data[0] << 0) |
		((int64_t)data[1] << 8) |
		((int64_t)data[2] << 16) |
		((int64_t)data[3] << 24) |
		((int64_t)data[4] << 32) |
		((int64_t)data[5] << 40) |
		((int64_t)data[6] << 48) |
		((int64_t)data[7] << 56)
	);
}
