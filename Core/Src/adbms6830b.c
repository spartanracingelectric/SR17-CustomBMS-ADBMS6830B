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
#include "module.h"
#include "spi.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

static const uint16_t AVERAGE_CELL_VOLTAGE_REGISTERS[6] = {RDACA, RDACB, RDACC, RDACD, RDACE, RDACF}; //command to read average from register
static const uint16_t REDUDANT_CELL_VOLTAGE_REGISTERS[6] = {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF};
static const uint16_t CLEAR_REGISTERS_COMMANDS[6] = {CLRCELL, CLRAUX, CLRFC, CLOVUV, CLRSPIN, CLRFLAG};
static const uint16_t AUX_REGISTERS[4] = { RDAUXA, RDAUXB, RDAUXC, RDAUXD };
static const uint16_t REDUNDANT_AUX_REGISTERS[4] = { RDRAXA, RDRAXB, RDRAXC, RDRAXD };
static const uint16_t ADBMS_CMD_RDSTAT[5] = {RDSTATA, RDSTATB, RDSTATC, RDSTATD, RDSTATE};

static uint16_t crc15Table[256];
static uint16_t crc10Table8Bit[256];
static uint16_t crc10Table6Bit[256];

DiagnosticPhase diagnosticPhase = DIAGNOSTIC_PHASE_REDUNDANT_START;

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
	ADBMS_generateCrcTables();
	ADBMS_csHigh();
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
	uint16_t command = ADCV | (redundantMode << 8) | (continuousMode << 7) | (dischargeMode << 4) | (filterResetMode << 2) | (openWireMode & 0x03);

	isoSPI_Idle_to_Ready();
	ADBMS_csLow();
	ADBMS_sendCommand(command);
	ADBMS_csHigh();
}

void ADBMS_startRedundantCellVoltageConversions(AdcContinuousMode continuousMode, AdcDischargeMode dischargeMode, AdcOpenWireMode openWireMode)
{
    uint16_t command = ADSV | (continuousMode << 7) | (dischargeMode << 4) | (openWireMode & 0x03);

	isoSPI_Idle_to_Ready();
	ADBMS_csLow();
	ADBMS_sendCommand(command);
	ADBMS_csHigh();
}


void ADBMS_startAuxConversions(AuxOpenWireMode openWireMode, AuxPullUpPinMode pullUpPinMode, AuxChannelSelect channelSelect) 
{
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
	//TODO: Error check
	HAL_SPI_Receive(&hspi1, (uint8_t*)&rxBuffer[0][0], NUM_MOD * (DATA_LEN + PEC_LEN), 100);
}

void ADBMS_sendCommand(uint16_t command) 
{
	uint8_t txBuffer[COMMAND_LENGTH + PEC_LEN];
	txBuffer[0] = (uint8_t)(command >> 8);
	txBuffer[1] = (uint8_t)(command);
	
	uint16_t pec = ADBMS_calculateCommandPec(txBuffer, COMMAND_LENGTH);
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
		bool isDataValid = ADBMS_checkDataPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);

		if (!isDataValid) 
		{
            moduleData[moduleIndex].pec_error = true; 
        } 
		else 
		{
            moduleData[moduleIndex].pec_error = false; 
        }
		
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
			int16_t rawVoltage = (int16_t)(((uint16_t)highByte << 8) | (uint16_t)lowByte);

			if (rawVoltage == (int16_t)DEFAULT_VOLTAGE_VALUE) 
			{
				moduleData[moduleIndex].cell_volt[cellIndex] = 0xFFFF;
			}
			else 
			{
				int32_t microVoltage = (int32_t)(1500000 + rawVoltage * 150);
				int16_t milliVoltage = (int16_t)(microVoltage / 1000);
				moduleData[moduleIndex].cell_volt[cellIndex] = milliVoltage;
				printf("Module %d, Cell %d, voltage %d\n", moduleIndex, cellIndex, milliVoltage);
			}
		}
	}
}

void ADBMS_getRedundantCellVoltages(ModuleData *moduleData) 
{
	uint8_t rxBuffer[NUM_MOD][REG_LEN];
	
	ADBMS_snap(); 
	int numberOfRegisters = (NUM_CELL_PER_MOD + (CELLS_PER_ADC_REGISTER - 1)) / CELLS_PER_ADC_REGISTER;
	for (uint8_t registerIndex = 0; registerIndex < numberOfRegisters; registerIndex++) 
	{
		isoSPI_Idle_to_Ready();
		ADBMS_csLow();
		ADBMS_sendCommand(REDUDANT_CELL_VOLTAGE_REGISTERS[registerIndex]);
		ADBMS_receiveData(rxBuffer);
		ADBMS_csHigh();

		ADBMS_parseCellVoltages(rxBuffer, registerIndex, moduleData);
	}
	ADBMS_unsnap();
}


void ADBMS_parseRedundantCellVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData) 
{
	uint8_t initialCellIndex = registerIndex * CELLS_PER_ADC_REGISTER;

	//Receive data from last module first
	for (int moduleIndex = NUM_MOD - 1; moduleIndex >= 0; moduleIndex--) 
	{
		bool isDataValid = ADBMS_checkDataPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);

		if (!isDataValid) {
            moduleData[moduleIndex].pec_error = true; 
        } 
		else 
		{
            moduleData[moduleIndex].pec_error = false; 
        }
		
		for (uint8_t cellOffset = 0; cellOffset < CELLS_PER_ADC_REGISTER; cellOffset++) 
		{
			uint8_t cellIndex = initialCellIndex + cellOffset;

			if (cellIndex > NUM_CELL_PER_MOD - 1) break;

			if (!isDataValid) 
			{
				moduleData[moduleIndex].redundantCellVoltage_mV[cellIndex] = 0xFFFF;
				continue;
			}

			uint8_t lowByte = rxBuffer[moduleIndex][2 * cellOffset];
			uint8_t highByte = rxBuffer[moduleIndex][2 * cellOffset + 1];
			uint16_t rawVoltage = (uint16_t)((highByte << 8) | lowByte);

			if (rawVoltage == 0x8000u) // Default value
			{
				moduleData[moduleIndex].redundantCellVoltage_mV[cellIndex] = 0xFFFF;
                continue;
			}

            uint32_t microVoltage = 1500000u + (uint32_t)rawVoltage * 150u;
            uint16_t milliVoltage = (uint16_t)(microVoltage / 1000u);

            if ((diagnosticPhase == DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_EVEN || diagnosticPhase == DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_ODD) && milliVoltage <= OPEN_WIRE_CHECK_VOLTAGE_MV)
            {
                //TODO: Add open wire fault
				moduleData[moduleIndex].redundantCellVoltage_mV[cellIndex] = 0xFFFF;
                continue;
            }

            moduleData[moduleIndex].redundantCellVoltage_mV[cellIndex] = milliVoltage;
		}
	}
}

void ADBMS_checkDiagnostics(ModuleData *moduleData)
{
    static uint32_t lastOpenWireCheck_ms = 0;
    uint32_t now_ms = HAL_GetTick();

    switch (diagnosticPhase) 
    {
        case DIAGNOSTIC_PHASE_REDUNDANT_START:
            ADBMS_startRedundantCellVoltageConversions(CONTINUOUS_MODE_ON, DISCHARGE_MODE_OFF, OPEN_WIRE_MODE_ALL_OFF);
            diagnosticPhase = DIAGNOSTIC_PHASE_REDUNDANT_RUNNING; 
            break;

        case DIAGNOSTIC_PHASE_REDUNDANT_RUNNING:
            ADBMS_getRedundantFaultFlags(moduleData);
            if (now_ms - lastOpenWireCheck_ms >= 1000)
            {
                diagnosticPhase = DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_EVEN;
                lastOpenWireCheck_ms = now_ms;
            }
            break;

        case DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_EVEN:
            ADBMS_startRedundantCellVoltageConversions(CONTINUOUS_MODE_OFF, DISCHARGE_MODE_OFF, OPEN_WIRE_MODE_EVEN_ON);
            ADBMS_getRedundantCellVoltages(moduleData);
            diagnosticPhase = DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_ODD;
            break;
        
        case DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_ODD:
            ADBMS_startRedundantCellVoltageConversions(CONTINUOUS_MODE_OFF, DISCHARGE_MODE_OFF, OPEN_WIRE_MODE_ODD_ON);
            ADBMS_getRedundantCellVoltages(moduleData);
            diagnosticPhase = DIAGNOSTIC_PHASE_REDUNDANT_START;
            break;
    }
}

void ADBMS_sendData(uint8_t data[NUM_MOD][DATA_LEN])  
{
	uint8_t txBuffer[NUM_MOD][DATA_LEN + PEC_LEN];
	
	for (int moduleIndex = 0; moduleIndex < NUM_MOD; moduleIndex++) 
	{
		memcpy(txBuffer[moduleIndex], data[moduleIndex], DATA_LEN);

		uint16_t dataPec = ADBMS_calculateDataPec(data[moduleIndex], DATA_LEN, 0);
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
		bool isDataValid = ADBMS_checkDataPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN + 0]);
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

	int numberOfRegisters = NUMBER_OF_AUX_REGISTERS;
	for (uint8_t registerIndex = 0; registerIndex < numberOfRegisters; registerIndex++) 
	{
		isoSPI_Idle_to_Ready();
		ADBMS_csLow();
		ADBMS_sendCommand(AUX_REGISTERS[registerIndex]);
		ADBMS_receiveData(rxBuffer);
		ADBMS_csHigh();

		ADBMS_parseGpioVoltages(rxBuffer, registerIndex, moduleData);
	}
	
}

void ADBMS_parseGpioVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData) 
{
	uint8_t initialGpioIndex = registerIndex * GPIOS_PER_AUX_REGISTER;

	//Receive data from last module first
	for (int moduleIndex = NUM_MOD - 1; moduleIndex >= 0; moduleIndex--) 
	{
		bool isDataValid = ADBMS_checkDataPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);
		
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
			uint16_t rawVoltageUnsigned = (uint16_t)((highByte << 8) | lowByte);
            int16_t rawVoltageSigned = (int16_t)((highByte << 8) | lowByte);
			//printf("Module: %d, GPIO: %d, Raw Voltage: %d\n", moduleIndex, gpioIndex, rawVoltage);


			if (rawVoltageUnsigned == 0x8000) // Default value
			{
				moduleData[moduleIndex].gpio_volt[gpioIndex] = 0xFFFF;
			}
			else 
			{
                int32_t microVoltageSigned = 1500000 + (int32_t)rawVoltageSigned * 150;
                int16_t milliVoltageSigned = (int16_t)(microVoltageSigned / 1000);
                
                if (milliVoltageSigned < 0 || milliVoltageSigned >= moduleData[moduleIndex].vref2)
                {
                    moduleData[moduleIndex].gpio_volt[gpioIndex] = 0xFFFF;
                }
                else 
                {
                    moduleData[moduleIndex].gpio_volt[gpioIndex] = milliVoltageSigned;
                }
				//printf("Module %d, GPIO %d, volt: %d\n", moduleIndex, gpioIndex + 1, milliVoltageSigned);
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
		bool isDataValid = ADBMS_checkDataPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);
		
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
			//printf("Module %d, vref: %d mv\n", moduleIndex, milliVoltage);
        }
	}
}

void ADBMS_getRedundantFaultFlags(ModuleData *moduleData)
{
	uint8_t rxBuffer[NUM_MOD][REG_LEN];

	isoSPI_Idle_to_Ready(); // Ensure link is up before transaction
	ADBMS_csLow();
    ADBMS_sendCommand(RDSTATC); 
    ADBMS_receiveData(rxBuffer);
	ADBMS_csHigh();
    ADBMS_parseRedundantFaultFlags(moduleData, rxBuffer);
}

void ADBMS_parseRedundantFaultFlags(ModuleData *moduleData, uint8_t rxBuffer[NUM_MOD][REG_LEN])
{
    for (int moduleIndex = NUM_MOD; moduleIndex >=0; moduleIndex--) 
    {
		bool isDataValid = ADBMS_checkDataPec(&rxBuffer[moduleIndex][0], DATA_LEN, &rxBuffer[moduleIndex][DATA_LEN]);
        if (!isDataValid) 
        {
            continue;
        }
        uint16_t lowByte = rxBuffer[moduleIndex][0];
        uint16_t highByte = rxBuffer[moduleIndex][1];
        uint16_t faultBits = (uint16_t)((highByte << 8) | lowByte);

        // loop through fault bits 
        for (uint8_t cellIndex = 0; cellIndex < NUM_CELL_PER_MOD; cellIndex++)
        {
            if (faultBits & (1 << cellIndex))  
            {
                //TODO: Set redundant fault flag
                moduleData[moduleIndex].cell_volt[cellIndex] = 0xFFFF;
            }
        }
    }
}


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

uint16_t ADBMS_calculateDataPec(uint8_t *data, int length, uint8_t commandCounter) 
{
	uint16_t crc = INITIAL_CRC_SEED;
	for (uint16_t i = 0; i < length; i++)
	{
        crc = (crc << 8) ^ crc10Table8Bit[((crc >> 2) ^ data[i]) & 0xFF];
	}
	crc = (crc << 6) ^ crc10Table6Bit[((crc >> 4) ^ commandCounter) & 0x3F];
	return crc &= 0x03FF;
}

uint16_t ADBMS_calculateCommandPec(uint8_t *data, int length)
{
	uint16_t crc = INITIAL_CRC_SEED;
	for (int i = 0; i < length; i++)
	{
		uint8_t index = (uint8_t)(crc >> 7) ^ data[i];
		crc = (crc << 8) ^ crc15Table[index];
	}
	return crc <<= 1;
}

bool ADBMS_checkDataPec(uint8_t *rxBuffer, uint16_t length, uint8_t pec[2])
{
	// Extract 6-bit command counter from the top 6 bits of pec[0]
    uint8_t commandCounter = (uint8_t)((pec[0] >> 2) & 0x3F);

    // Extract 10-bit CRC from the bottom 2 bits of pec[0] and all 8 bits of pec[1]
    uint16_t receivedCrc = (uint16_t)(((pec[0] & 0x03) << 8) | pec[1]);  

    uint16_t calculatedCrc = ADBMS_calculateDataPec(rxBuffer, length, commandCounter);

    return (receivedCrc == calculatedCrc);
}

void ADBMS_generateCrcTables(void)
{
	ADBMS_generateCrc15Table();
	ADBMS_generateCrc10Table8Bit();
	ADBMS_generateCrc10Table6Bit();
}

void ADBMS_generateCrc15Table(void)
{
	for (uint16_t i = 0; i < 256; i++)
	{
		uint16_t crc = i << 7;		
		for (int bit = 0; bit < 8; bit++)
		{
			if (crc & 0x4000) // MSB of 15-bit CRC (bit 14)
			{       
				crc = (uint16_t)((crc << 1) ^ COMMAND_CRC_POLYNOMIAL);
			}
			else
			{
				crc <<= 1;
			}
		}
		crc15Table[i] = crc & 0x7FFF;
	}
}

void ADBMS_generateCrc10Table6Bit(void)
{
    for (uint16_t i = 0; i < 64; ++i)
    {
        uint16_t crc = (uint16_t)(i << 4);

        // Clock 6 bits through the LFSR
        for (int bit = 0; bit < 6; ++bit)
        {
            if (crc & 0x200u) // MSB of 10-bit CRC
			{
                crc = (uint16_t)((crc << 1) ^ DATA_CRC_POLYNOMIAL);
			}
            else
			{
                crc <<= 1;
			}
        }
        crc10Table6Bit[i] = (uint16_t)(crc & 0x03FFu);   // keep 10 bits
    }
}

void ADBMS_generateCrc10Table8Bit(void)
{
	for (uint16_t i = 0; i < 256; ++i)
	{
		uint16_t crc = i << 2;
		
		for (int bit = 0; bit < 8; ++bit)
		{
			if (crc & 0x200) 
			{
				crc = (uint16_t)((crc << 1) ^ DATA_CRC_POLYNOMIAL);
			}
			else 
			{
				crc <<= 1;
			}
		}
		crc10Table8Bit[i] = (uint16_t)(crc & 0x03FF);
	}
}

