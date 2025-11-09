/** @file adbms.c
 *  @brief Host-side helpers for ADBMS68xx/LTC6811 battery monitor chains over isoSPI/SPI.
 *
 *  @details
 *  - Powers/wakes the daisy chain and issues commands (ADCV/ADSV/SNAP/UNSNAP/etc.).
 *  - Reads cell averages (RDACx), GPIO/AUX pages, and 6-byte Serial IDs (SID).
 *  - Writes PWM/CFG/COMM register groups to all devices in a chain.
 *  - Verifies link and payload integrity using command PEC15 and data PEC10.
 *
 *  @note Requires:
 *    - HAL SPI handle `hspi1`
 *    - Chip select helpers `ADBMS_nCS_Low()` / `ADBMS_nCS_High()`
 *    - Topology macros: `NUM_MOD`, `NUM_CELL_PER_MOD`
 *  @warning All functions are blocking and not thread-safe.
 */
#include <adbms6830b.h>
#include "spi.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>



static const uint16_t AVERAGE_CELL_VOLTAGE_REGISTERS[6] = {RDACA, RDACB, RDACC, RDACD, RDACE, RDACF}; //command to read average from register
static const uint16_t CLEAR_REGISTERS_COMMANDS[6] = {CLRCELL, CLRAUX, CLRFC, CLOVUV, CLRSPIN, CLRFLAG};
static const uint16_t AUX_REGISTERS[4] = { RDAUXA, RDAUXB, RDAUXC, RDAUXD };
static const uint16_t ADBMS_CMD_RDSTAT[5] = {RDSTATA, RDSTATB, RDSTATC, RDSTATD, RDSTATE};

/**
 * @brief Wake the ADBMS/LTC isoSPI interface from IDLE to READY by clocking 0xFF.
 *
 * Some ADBMS/LTC parts require a short SPI activity (while nCS is low) to exit IDLE.
 * Sending 0xFF with nCS asserted is a safe way to provide clocks without issuing a command.
 */
void isoSPI_Idle_to_Ready() {
	uint8_t hex_ff = 0xFF;
	ADBMS_csLow();							   // Assert CS to address the chain
	HAL_SPI_Transmit(&hspi1, &hex_ff, 1, 100);     // Send a dummy byte to toggle SCK and wake isoSPI
	ADBMS_csHigh();							   // Deassert CS
	HAL_Delay(1);                                  // Small guard delay to ensure READY state
}

/**
 * @brief Wake devices from SLEEP by toggling nCS (no clocks required in sleep wake).
 *
 * Many LTC/ADBMS devices detect wake on nCS edges while asleep.
 * Two low-high toggles with small delays are commonly recommended.
 */
void ADBMS_wakeUp() {
    for (int i = 0; i < 2; i++) {
        ADBMS_csLow();
        HAL_Delay(1);
        ADBMS_csHigh();
        HAL_Delay(1);
    }
}

void ADBMS_clearRegisters()
{
	isoSPI_Idle_to_Ready();   
	size_t numberOfCommands = sizeof(CLEAR_REGISTERS_COMMANDS) / sizeof(CLEAR_REGISTERS_COMMANDS[0]);
	for(int i = 0; i < numberOfCommands; i++)
	{
		ADBMS_csLow();
		ADBMS_sendCommand(CLEAR_REGISTERS_COMMANDS[i]);
		ADBMS_csHigh();
	}
}

/**
 * @brief Basic init sequence: wake from sleep, exit SNAP mode, and start cell ADC.
 *
 * Order:
 *  1) Ensure devices are awake.
 *  2) UNSNAP so that live registers are accessible.
 *  3) Start voltage conversions (continuous or as configured by your fields).
 */
void ADBMS_init()
{
	ADBMS_wakeUp();
	ADBMS_unsnap();
	ADBMS_clearRegisters();
	ADBMS_startCellVoltageConversions(REDUNDANT_MODE_OFF, CONTINUOUS_MODE_ON, DISCHARGE_MODE_OFF, FILTER_RESET_MODE_ON, OPEN_WIRE_MODE_ALL_OFF);
}

/**
 * @brief Start cell-voltage conversion by emitting the packed ADCV command.
 *
 * The command is built bit-by-bit into a 11-bit (here packed into 16-bit) command word:
 *  bit10: 0 (fixed)
 *  bit9 : 1 (fixed)
 *  bit8 : RD   (redundant samples / data rate select; enum AdcRD)
 *  bit7 : CONT (continuous conversion enable; enum AdcCONT)
 *  bit6 : 1 (fixed)
 *  bit5 : 1 (fixed)
 *  bit4 : DCP  (discharge-permit during conversion; enum AdcDCP)
 *  bit3 : 0 (fixed)
 *  bit2 : RSTF (reset filter/averager; enum AdcRSTF)
 *  bit1:0: OW[1:0] (open-wire test mode; enum AdcOW)
 *
 * After packing, compute PEC15 over the 2 command bytes and transmit 4 bytes total.
 */
void ADBMS_startCellVoltageConversions(AdcRedundantMode redundantMode, AdcContinuousMode continuousMode, AdcDischargeMode dischargeMode, AdcFilterResetMode filterResetMode, AdcOpenWireMode openWireMode)
{
	uint16_t command = 0;

	command |= (0 << 10);    					// bit10 = 0        (fixed)
	command |= (1 << 9);     					// bit9  = 1        (fixed sync bit)
	command |= (redundantMode << 8);     		// bit8  = RD       (rate/redundancy)
	command |= (continuousMode << 7);     		// bit7  = CONT     (continuous mode)
	command |= (1 << 6);     					// bit6  = 1        (fixed)
	command |= (1 << 5);     					// bit5  = 1        (fixed)
	command |= (dischargeMode << 4);     		// bit4  = DCP      (balance during conv)
	command |= (0 << 3);     					// bit3  = 0        (fixed)
	command |= (filterResetMode << 2);     		// bit2  = RSTF     (reset digital filter)
	command |= (openWireMode & 0x03);     		// bit1-0 = OW[1:0] (open-wire mode)

	isoSPI_Idle_to_Ready();
	ADBMS_csLow();
	ADBMS_sendCommand(command);
	ADBMS_csHigh();
}

void ADBMS_startAuxConversions(AuxOpenWireMode openWireMode, AuxPullUpPinMode pullUpPinMode, AuxChannelSelect channelSelect) {
    uint16_t command = ADAX | (openWireMode << 8) | (pullUpPinMode << 7) | channelSelect;

	isoSPI_Idle_to_Ready();
    ADBMS_csLow();
    ADBMS_sendCommand(command);
    ADBMS_csHigh();
}

/**
 * @brief Issue SNAP command to latch a coherent “snapshot” of measurement registers.
 *
 * SNAP freezes a consistent set of measurement results so you can read them across
 * multiple RDCV/RDAUX/etc. transactions without internal updates racing your reads.
 * Typical flow: SNAP -> read all needed registers -> UNSNAP when done.
 */
void ADBMS_snap()
{
	isoSPI_Idle_to_Ready(); 
	ADBMS_csLow();
	ADBMS_sendCommand(SNAP);
	ADBMS_csHigh();
}

/**
 * @brief Issue UNSNAP command to resume live register updates after a SNAP.
 *
 * Call this once you’ve finished reading all latched measurement pages.
 */
void ADBMS_unsnap()
{
	isoSPI_Idle_to_Ready();   // Ensure link is awake
	ADBMS_csLow();
	ADBMS_sendCommand(UNSNAP);
	ADBMS_csHigh();
}

void ADBMS_receiveData(uint8_t rxBuffer[NUM_MOD][DATA_LEN + PEC_LEN])
{
	//TODO: Verify PEC
	HAL_SPI_Receive(&hspi1, (uint8_t*)&rxBuffer[0][0], NUM_MOD * (DATA_LEN + PEC_LEN), 100);
}

void ADBMS_sendCommand(uint16_t command) 
{
	uint8_t txBuffer[COMMAND_LENGTH + PEC_LEN];
	txBuffer[0] = (uint8_t)(command >> 8);
	txBuffer[1] = (uint8_t)(command);
	
	uint16_t pec = ADBMS_calcPec15(txBuffer, 2);
	txBuffer[2] = (uint8_t)(pec >> 8);
	txBuffer[3] = (uint8_t)(pec);

	HAL_SPI_Transmit(&hspi1, txBuffer, COMMAND_LENGTH + PEC_LEN, 100);
}

/**
 * @brief Read averaged cell voltages for all modules in the daisy chain.
 *
 * Flow:
 *  1) SNAP to latch a coherent set of voltage registers across modules.
 *  2) For each RDAC page (RDCV group), send RDAC command and receive data.
 *  3) For each device (“module”) in the chain:
 *     - Verify RX PEC10.
 *     - Unpack 12-bit/16-bit raw words (device-specific width) from the 6-byte data block.
 *     - Convert raw counts to millivolts (here: 1.5 V offset + 0.150 mV/LSB).
 *     - Store into mod[devIndex].cell_volt[cellInMod].
 *  4) UNSNAP to resume live updates.
 *
 * Notes:
 *  - RX_BYTES_PER_IC = DATA_LEN + PEC_LEN (6 + 2 = 8 bytes per device per read).
 *  - The loop over regIndex covers however many cell groups are mapped per RDAC page.
 *  - A raw value of 0x8000 indicates “invalid/reset” per the device convention; we store 0xFFFF.
 *  - PEC failures are logged and that device’s data for the page is skipped.
 *
 * @param[out] mod  Array of ModuleData; each entry receives cell voltages in mV.
 * @return LTC_SPI_StatusTypeDef  Bitfield with TX/RX error flags set on HAL failures.
 */
void ADBMS_getCellVoltages(ModuleData *moduleData) 
{
	uint8_t rxBuffer[NUM_MOD][REG_LEN];
	
	ADBMS_snap(); 
	int numberOfRegisters = (NUM_CELL_PER_MOD + (CELLS_PER_ADC_REGISTER - 1)) / CELLS_PER_ADC_REGISTER;
	for (uint8_t registerIndex = 0; registerIndex < numberOfRegisters; registerIndex++) 
	{
		isoSPI_Idle_to_Ready();
		ADBMS_csLow();
		ADBMS_sendCommand(AVERAGE_CELL_VOLTAGE_REGISTERS[registerIndex]);
		ADBMS_receiveData(rxBuffer);
		ADBMS_csHigh();

		ADBMS_parseCellVoltages(rxBuffer, registerIndex, moduleData);
	}
	ADBMS_unsnap();
}

void ADBMS_parseCellVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData) 
{
	uint8_t initialCellIndex = registerIndex * CELLS_PER_ADC_REGISTER;

	//Receive data from last module first
	for (int moduleIndex = NUM_MOD - 1; moduleIndex >= 0; moduleIndex--) 
	{
		bool isDataValid = ADBMS_checkRxPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);
		
		for (uint8_t cellOffset = 0; cellOffset < CELLS_PER_ADC_REGISTER; cellOffset++) 
		{
			uint8_t cellIndex = initialCellIndex + cellOffset;

			if (cellIndex > NUM_CELL_PER_MOD - 1) break;

			if (!isDataValid) 
			{
				moduleData[moduleIndex].cell_volt[cellIndex] = 0xFFFF;
				continue;
			}

			uint8_t lowByte = rxBuffer[moduleIndex][2 * cellOffset];
			uint8_t highByte = rxBuffer[moduleIndex][2 * cellOffset + 1];
			uint16_t rawVoltage = (uint16_t)((highByte << 8) | lowByte);

			if (rawVoltage == 0x8000u) //Default value
			{
				moduleData[moduleIndex].cell_volt[cellIndex] = 0xFFFF;
			}
			else 
			{
				uint32_t microVoltage = 1500000u + (uint32_t)rawVoltage * 150u;
				uint16_t milliVoltage = (uint16_t)(microVoltage / 1000u);
				moduleData[moduleIndex].cell_volt[cellIndex] = milliVoltage;
			}
		}
	}
}

void ADBMS_sendData(uint8_t data[NUM_MOD][DATA_LEN])  
{
	uint8_t txBuffer[NUM_MOD][DATA_LEN + PEC_LEN];
	
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		memcpy(txBuffer[moduleIndex], data[moduleIndex], DATA_LEN);

		uint16_t dataPec = ADBMS_calcPec10(data[moduleIndex], DATA_LEN, NULL);
		txBuffer[moduleIndex][DATA_LEN + 0] = (uint8_t)(dataPec >> 8);
		txBuffer[moduleIndex][DATA_LEN + 1] = (uint8_t)(dataPec);
	}

	HAL_SPI_Transmit(&hspi1, (uint8_t*)&txBuffer, (DATA_LEN + PEC_LEN) * NUM_MOD, 100);
}

void ADBMS_writeConfigurationRegisterB(BalanceStatus *blst) 
{
	uint8_t txBuffer[NUM_MOD][DATA_LEN];
	uint8_t CFGBR0 = (uint8_t)(VUV & 0xFF); // VUV[7:0]
	uint8_t CFGBR1 = (uint8_t)(((VUV >> 8) & 0x0F) << 4 | (VOV & 0x0F)); // VUV[11:8], VOV[3:0]
    uint8_t CFGBR2 = (uint8_t)((VOV >> 4) & 0xFF); // VOV[11:4];
    uint8_t CFGBR3 = 0x00; //settings for discharge timer
	// Pack command bits into the register CFGBR4
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
	{
		uint8_t CFGBR4 = 0;
		uint8_t CFGBR5 = 0;
		for (int cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++) 
		{
			if(cellIndex < 8)
			{
				CFGBR4 |= (blst[moduleIndex].cellsToBalance[cellIndex] & 0x01) << cellIndex;
			}
			else
			{
				CFGBR5 |= (blst[moduleIndex].cellsToBalance[cellIndex] & 0x01) << (cellIndex - 8);
			}
		}
		txBuffer[moduleIndex][0] = CFGBR0;
		txBuffer[moduleIndex][1] = CFGBR1;
		txBuffer[moduleIndex][2] = CFGBR2;
		txBuffer[moduleIndex][3] = CFGBR3;
		txBuffer[moduleIndex][4] = CFGBR4;
		txBuffer[moduleIndex][5] = CFGBR5;
	}
	isoSPI_Idle_to_Ready();
	ADBMS_csLow();
	ADBMS_sendCommand(WRCFGB);
	ADBMS_sendData(txBuffer);
	ADBMS_csHigh();
}

void ADBMS_readConfigurationRegisterB(ConfigurationRegisterB *configB) 
{
	uint8_t rxBuffer[NUM_MOD][DATA_LEN + PEC_LEN];

	isoSPI_Idle_to_Ready(); // Ensure link is up before transaction
	ADBMS_csLow();
	ADBMS_sendCommand(RDCFGB);
	ADBMS_receiveData(rxBuffer);
	ADBMS_csHigh();

	for (uint8_t moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		bool isDataValid = ADBMS_checkRxPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN + 0]);
		if (!isDataValid) 
		{
			//TODO: Add proper error state
			continue; 
		}
		ADBMS_parseConfigurationRegisterB(rxBuffer[moduleIndex], &configB[moduleIndex]);
	}
}

void ADBMS_parseConfigurationRegisterB(uint8_t data[DATA_LEN], ConfigurationRegisterB *configB) 
{
	//TODO: Convert VUV and VOV into voltage
	configB->underVoltageThreshold_V = (uint16_t)data[0] | (uint16_t)((data[1] & 0x0Fu) << 8);
	configB->overVoltageThreshold_V = (uint16_t)((data[1] >> 4) & 0x0Fu) | (uint16_t)(data[2] << 4);
	configB->cellsDischargeStatus = (uint16_t)data[4] | (uint16_t)(data[5] << 8);
}

void ADBMS_getGpioVoltages(ModuleData *moduleData) 
{
	uint8_t rxBuffer[NUM_MOD][REG_LEN];

	int numberOfRegisters = (NUM_CELL_PER_MOD + (GPIOS_PER_AUX_REGISTER - 1)) / GPIOS_PER_AUX_REGISTER;
	for (uint8_t registerIndex = 0; registerIndex < numberOfRegisters; registerIndex++) 
	{
		isoSPI_Idle_to_Ready();
		ADBMS_csLow();
		ADBMS_sendCommand(AUX_REGISTERS[registerIndex]);
		ADBMS_receiveData(rxBuffer);
		ADBMS_csHigh();

		ADBMS_parseGpioVoltages(rxBuffer, registerIndex, moduleData);
	}
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++)
    {
        printf("Module %d GPIO Voltages:\n", moduleIndex);
        
        for (int gpioIndex = 0; gpioIndex < GPIOS_PER_IC; gpioIndex++)
        {
            uint16_t voltage = moduleData[moduleIndex].gpio_volt[gpioIndex];
            
            if (voltage == 0xFFFF)
            {
                printf("  GPIO%d: INVALID\n", gpioIndex);
            }
            else
            {
                printf("  GPIO%d: %u mV\n", gpioIndex, voltage);
            }
        }
        printf("\n");
    }
}

void ADBMS_parseGpioVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData) 
{
	uint8_t initialGpioIndex = registerIndex * GPIOS_PER_AUX_REGISTER;

	//Receive data from last module first
	for (int moduleIndex = NUM_MOD - 1; moduleIndex >= 0; moduleIndex--) 
	{
		bool isDataValid = ADBMS_checkRxPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);
		
		for (uint8_t gpioOffset = 0; gpioOffset < GPIOS_PER_AUX_REGISTER; gpioOffset++) 
		{
			uint8_t gpioIndex = initialGpioIndex + gpioOffset;

			if (gpioIndex > GPIOS_PER_IC - 1) break;

			if (!isDataValid) 
			{
				moduleData[moduleIndex].gpio_volt[gpioIndex] = 0xFFFF;
				continue;
			}

			uint8_t lowByte = rxBuffer[moduleIndex][2 * gpioOffset];
			uint8_t highByte = rxBuffer[moduleIndex][2 * gpioOffset + 1];
			uint16_t rawVoltage = (uint16_t)((highByte << 8) | lowByte);

			if (rawVoltage == 0x8000u) //Default value
			{
				moduleData[moduleIndex].gpio_volt[gpioIndex] = 0xFFFF;
			}
			else 
			{
				uint32_t microVoltage = 1500000u + (uint32_t)rawVoltage * 150u;
				uint16_t milliVoltage = (uint16_t)(microVoltage / 1000u);
				moduleData[moduleIndex].gpio_volt[gpioIndex] = milliVoltage;
			}
		}
	}
}

void ADBMS_getVref2(ModuleData *moduleData) {
	uint8_t rxBuffer[NUM_MOD][REG_LEN];

	isoSPI_Idle_to_Ready(); // Ensure link is up before transaction
	ADBMS_csLow();
    ADBMS_sendCommand(RDSTATA); 
    ADBMS_receiveData(rxBuffer);
	ADBMS_csHigh();
    ADBMS_parseVref2Voltages(rxBuffer, moduleData);
}

void ADBMS_parseVref2Voltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], ModuleData *moduleData) 
{
	//Receive data from last module first
	for (int moduleIndex = NUM_MOD - 1; moduleIndex >= 0; moduleIndex--) 
	{
		bool isDataValid = ADBMS_checkRxPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);
		
        if (!isDataValid) 
        {
            moduleData[moduleIndex].vref2 = 0xFFFF;
            continue;
        }

        uint8_t lowByte = rxBuffer[moduleIndex][0];
        uint8_t highByte = rxBuffer[moduleIndex][1];
        uint16_t rawVoltage = (uint16_t)((highByte << 8) | lowByte);

        if (rawVoltage == 0x8000u) //Default value
        {
            moduleData[moduleIndex].vref2 = 0xFFFF;
        }
        else 
        {
            uint32_t microVoltage = 1500000u + (uint32_t)rawVoltage * 150u;
            uint16_t milliVoltage = (uint16_t)(microVoltage / 1000u);
            moduleData[moduleIndex].vref2 = milliVoltage;
        }
	}
}

/** CRC15 lookup table for command PEC15 (Linear/ADI LTC/ADBMS family).
 *  - Polynomial: 0x4599 (x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
 *  - Seed: 0x0010 (decimal 16)
 *  - Implementation detail: returned value is left-shifted by 1 so LSB is 0.
 */
const uint16_t Crc15Table[256] =
{
    0x0000, 0xc599, 0xceab, 0xb32,  0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x2be,  0xc727, 0xcc15, 0x98c,  0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2,  0xc25b, 0xc969, 0xcf0,  0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x57c,  0xc0e5, 0xcbd7, 0xe4e,  0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0xf84,  0xca1d, 0xc12f, 0x4b6,  0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a,
	0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2,
	0xe46b, 0xef59, 0x2ac0, 0xd3a,  0xc8a3, 0xc391, 0x608,  0xd5f5, 0x106c, 0x1b5e, 0xdec7,
	0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06,
	0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80,
	0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41,
	0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x846,  0xcddf, 0xc6ed, 0x374,
	0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8,  0xcf61, 0xc453, 0x1ca,  0xd237, 0x17ae, 0x1c9c,
	0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b, 0x2d02, 0xa76f, 0x62f6,
	0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7,
	0x4e3e, 0x450c, 0x8095
};

/** CRC10 lookup table for data PEC10 (used by RX frames from devices).
 *  - Polynomial: x^10 + x^7 + x^3 + x^2 + x + 1
 *  - Represented here with helper constant 0x08F during the final bit-walk.
 *  - Seed: 0x0010 (decimal 16)
 */
static const uint16_t crc10Table[256] =
{
    0x000, 0x08f, 0x11e, 0x191, 0x23c, 0x2b3, 0x322, 0x3ad, 0x0f7, 0x078, 0x1e9, 0x166, 0x2cb, 0x244, 0x3d5, 0x35a,
    0x1ee, 0x161, 0x0f0, 0x07f, 0x3d2, 0x35d, 0x2cc, 0x243, 0x119, 0x196, 0x007, 0x088, 0x325, 0x3aa, 0x23b, 0x2b4,
    0x3dc, 0x353, 0x2c2, 0x24d, 0x1e0, 0x16f, 0x0fe, 0x071, 0x32b, 0x3a4, 0x235, 0x2ba, 0x117, 0x198, 0x009, 0x086,
    0x232, 0x2bd, 0x32c, 0x3a3, 0x00e, 0x081, 0x110, 0x19f, 0x2c5, 0x24a, 0x3db, 0x354, 0x0f9, 0x076, 0x1e7, 0x168,
    0x337, 0x3b8, 0x229, 0x2a6, 0x10b, 0x184, 0x015, 0x09a, 0x3c0, 0x34f, 0x2de, 0x251, 0x1fc, 0x173, 0x0e2, 0x06d,
    0x2d9, 0x256, 0x3c7, 0x348, 0x0e5, 0x06a, 0x1fb, 0x174, 0x22e, 0x2a1, 0x330, 0x3bf, 0x012, 0x09d, 0x10c, 0x183,
    0x0eb, 0x064, 0x1f5, 0x17a, 0x2d7, 0x258, 0x3c9, 0x346, 0x01c, 0x093, 0x102, 0x18d, 0x220, 0x2af, 0x33e, 0x3b1,
    0x105, 0x18a, 0x01b, 0x094, 0x339, 0x3b6, 0x227, 0x2a8, 0x1f2, 0x17d, 0x0ec, 0x063, 0x3ce, 0x341, 0x2d0, 0x25f,
    0x2e1, 0x26e, 0x3ff, 0x370, 0x0dd, 0x052, 0x1c3, 0x14c, 0x216, 0x299, 0x308, 0x387, 0x02a, 0x0a5, 0x134, 0x1bb,
    0x30f, 0x380, 0x211, 0x29e, 0x133, 0x1bc, 0x02d, 0x0a2, 0x3f8, 0x377, 0x2e6, 0x269, 0x1c4, 0x14b, 0x0da, 0x055,
    0x13d, 0x1b2, 0x023, 0x0ac, 0x301, 0x38e, 0x21f, 0x290, 0x1ca, 0x145, 0x0d4, 0x05b, 0x3f6, 0x379, 0x2e8, 0x267,
    0x0d3, 0x05c, 0x1cd, 0x142, 0x2ef, 0x260, 0x3f1, 0x37e, 0x024, 0x0ab, 0x13a, 0x1b5, 0x218, 0x297, 0x306, 0x389,
    0x1d6, 0x159, 0x0c8, 0x047, 0x3ea, 0x365, 0x2f4, 0x27b, 0x121, 0x1ae, 0x03f, 0x0b0, 0x31d, 0x392, 0x203, 0x28c,
    0x038, 0x0b7, 0x126, 0x1a9, 0x204, 0x28b, 0x31a, 0x395, 0x0cf, 0x040, 0x1d1, 0x15e, 0x2f3, 0x27c, 0x3ed, 0x362,
    0x20a, 0x285, 0x314, 0x39b, 0x036, 0x0b9, 0x128, 0x1a7, 0x2fd, 0x272, 0x3e3, 0x36c, 0x0c1, 0x04e, 0x1df, 0x150,
    0x3e4, 0x36b, 0x2fa, 0x275, 0x1d8, 0x157, 0x0c6, 0x049, 0x313, 0x39c, 0x20d, 0x282, 0x12f, 0x1a0, 0x031, 0x0be
};

/**
 * @brief Read 6-byte Serial IDs (SID) from each module in the daisy chain.
 *
 * Protocol:
 *  1) Build RDSID command and append PEC15 (cmd bytes only).
 *  2) Transmit command once; then receive (6 data + 2 PEC10) bytes per module.
 *  3) For each module:
 *     - Verify RX PEC10.
 *     - If OK, copy 6-byte SID to read_sid[devIndex][] and print it.
 *
 * @param[out] read_sid  [NUM_MOD][DATA_LEN] destination for per-module 6-byte SIDs.
 * @return LTC_SPI_StatusTypeDef with TX/RX error bits set on HAL failures (PEC errors are logged only).
 */
void ADBMS_ReadSID(ModuleData *mod) {
	uint8_t rxBuffer[NUM_MOD][DATA_LEN + PEC_LEN];  // Concatenated receive buffer for the whole chain

	//TODO: Finish
    isoSPI_Idle_to_Ready();

    ADBMS_csLow();
	ADBMS_sendCommand(RDSID);
	ADBMS_receiveData(rxBuffer);
    ADBMS_csHigh();

//     // 3) Validate and unpack each module's payload
//     for (uint8_t Index = 0; devIndex < NUM_MOD; devIndex++) {
//         uint16_t offset = (uint16_t)(devIndex * RX_BYTES_PER_IC);

//         uint8_t *sid    = &rx_buffer[offset];            // 6-byte SID
//         uint8_t *sidpec = &rx_buffer[offset + DATA_LEN]; // 2-byte PEC10 (packed with CC)

//         // Verify 10-bit PEC computed over the 6 SID bytes (+ command counter if present)
//         bool pec10Check = ADBMS_checkRxPec(sid, DATA_LEN, sidpec);
//         if (!pec10Check) {
// //        	printf("M%d failed\n", devIndex + 1);
//             continue; // Skip copying this module’s SID if PEC fails
//         }
//         memcpy(mod[devIndex].sid, sid, DATA_LEN);
//     }
}

/**
 * @brief Compute PEC15 (CRC15) for command frames using a 256-entry lookup table.
 *
 * Details:
 *  - Seed (remainder) starts at 0x0010.
 *  - Each input byte indexes into Crc15Table using (remainder >> 7) ^ data[i].
 *  - The final remainder is left-shifted by 1 (LSB = 0 per LTC/ADI convention).
 *
 * @param data Pointer to the bytes over which PEC15 is calculated (typically 2-byte command).
 * @param len  Number of bytes included in the PEC calculation.
 * @return 16-bit PEC value (with LSB cleared).
 */
uint16_t ADBMS_calcPec15(uint8_t *data, uint8_t len)
{
    uint16_t remainder, addr;
    remainder = 16; // PEC seed = 0x0010
    for (uint8_t i = 0; i < len; i ++)
    {
    	// Table index uses top 9 bits of remainder (>>7 gives 9 bits but we AND 0xFF for 8-bit index here)
        addr = (((remainder >> 7) ^ data[i]) & 0xff);
        remainder = ((remainder<<8)^Crc15Table[addr]);
    }
    // CRC15 for LTC devices is 15-bit with a zero LSB; multiply by 2 to force bit0 = 0
    return(remainder*2);
}

/**
 * @brief Compute PEC10 (CRC10) used on received data blocks from the device.
 *
 * The device appends a 10-bit CRC (plus a 6-bit command counter in the same 2-byte field).
 * We first run bytes through a 256-entry table (seed = 0x0010), then (optionally) fold in
 * the 6-bit command counter (CC) by XORing it into the top bits, and finally “walk” 6 bits
 * to align the 10-bit remainder (per datasheet convention).
 *
 * Polynomial: x^10 + x^7 + x^3 + x^2 + x + 1.
 *
 * @param pDataBuf        Pointer to received data bytes to check (e.g., 6-byte SID or 6-byte voltage page).
 * @param nLength         Number of bytes to include in the CRC calculation.
 * @param commandCounter  Optional pointer to a 6-bit CC value (0..63). Pass NULL if not used.
 * @return 10-bit CRC value in the low bits of the return (mask with 0x03FF if needed).
 */
uint16_t ADBMS_calcPec10(uint8_t *pDataBuf, int nLength, uint8_t *commandCounter)
{
    uint16_t nRemainder = 16u; // Seed 0x0010
    uint16_t nPolynomial = 0x8Fu; // Helper constant used during final 6-bit modulation
    uint8_t nByteIndex, nBitIndex;
    uint16_t nTableAddr;

    // Table-driven accumulation over data bytes
    for (nByteIndex = 0u; nByteIndex < nLength; ++nByteIndex)
    {
    	// Index = ((remainder >> 2) ^ dataByte) & 0xFF
        nTableAddr = (uint16_t)(((uint16_t)(nRemainder >> 2) ^ (uint8_t)pDataBuf[nByteIndex]) & (uint8_t)0xff);
        nRemainder = (uint16_t)(((uint16_t)(nRemainder << 8)) ^ crc10Table[nTableAddr]);
    }

    // Some frames encode a 6-bit command counter (CC) alongside CRC10 in the 2-byte PEC field.
    // If caller provides CC, fold it in before the final 6-bit “bit-walk.”
    if (commandCounter != NULL)
    {
        nRemainder ^= (uint16_t)(*commandCounter << 4u);
    }

    // Finish with a 6-bit modulo-2 division to land on the 10-bit remainder
    for (nBitIndex = 6u; nBitIndex > 0u; --nBitIndex)
    {
        if ((nRemainder & 0x200u) > 0u) // If MSB (bit9) is set
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
            nRemainder = (uint16_t)(nRemainder ^ nPolynomial);
        }
        else
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
        }
    }

    // Return only the 10 valid bits
    return ((uint16_t)(nRemainder & 0x3FFu));
}

/**
 * @brief Check a received frame’s PEC10 against the two PEC bytes from the device.
 *
 * The device’s 2-byte PEC field packs:
 *   - Bits [15:10]: 6-bit command counter (CC)
 *   - Bits [9:0]  : 10-bit CRC (DPEC)
 *
 * We extract CC and DPEC, recompute CRC10 over the data (including CC),
 * and compare the 10 LSBs.
 *
 * @param rxBuffer Pointer to the received data bytes (e.g., 6-byte SID or 6-byte data block).
 * @param len      Number of data bytes.
 * @param pec      Pointer to the two PEC bytes from the device (big-endian: pec[0] first, pec[1] second).
 * @return true if CRC matches; false on mismatch.
 */
bool ADBMS_checkRxPec(const uint8_t *rxBuffer, int len, const uint8_t pec[2])
{
	// Extract 6-bit command counter from the top 6 bits of pec[0]
    uint8_t  cc   = (uint8_t)((pec[0] >> 2) & 0x3F);

    // Extract 10-bit CRC from the bottom 2 bits of pec[0] and all 8 bits of pec[1]
    uint16_t dpec = (uint16_t)(((pec[0] & 0x03) << 8) | pec[1]);  // 10bit

    // Recompute CRC10 over rxBuffer, including the same CC if present
    uint16_t calc = (uint16_t)(ADBMS_calcPec10((uint8_t*)rxBuffer, len, &cc) & 0x03FF);

    // Compare only the 10 LSBs
    return ((dpec & 0x03FF) == calc);
}
