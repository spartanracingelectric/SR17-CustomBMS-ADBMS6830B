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
#include <ADBMS.h>
#include "spi.h"
#include "main.h"
#include <stdio.h>

uint8_t wrpwm_buffer[4 + (8 * NUM_MOD)];
uint8_t wrcfg_buffer[4 + (8 * NUM_MOD)];
uint8_t wrcomm_buffer[4 + (8 * NUM_MOD)];
const uint8_t RDSID[2] = { 0x00, 0x2C };
const uint8_t SNAP[2] = {0x00, 0x2D};
const uint8_t UNSNAP[2] = {0x00, 0x2F};

static const uint16_t ADBMS_CMD_RDAC[6] = { RDACA, RDACB, RDACC, RDACD, RDACE, RDACF}; //command to read average from register

static const uint16_t LTC_CMD_AUXREG[2] = { LTC_CMD_RDAUXA, LTC_CMD_RDAUXB };

/**
 * @brief Wake the ADBMS/LTC isoSPI interface from IDLE to READY by clocking 0xFF.
 *
 * Some ADBMS/LTC parts require a short SPI activity (while nCS is low) to exit IDLE.
 * Sending 0xFF with nCS asserted is a safe way to provide clocks without issuing a command.
 */
void isoSPI_Idle_to_Ready(void) {
	uint8_t hex_ff = 0xFF;
	ADBMS_nCS_Low();							   // Assert CS to address the chain
	HAL_SPI_Transmit(&hspi1, &hex_ff, 1, 100);     // Send a dummy byte to toggle SCK and wake isoSPI
	ADBMS_nCS_High();							   // Deassert CS
	HAL_Delay(1);                                  // Small guard delay to ensure READY state
}

/**
 * @brief Wake devices from SLEEP by toggling nCS (no clocks required in sleep wake).
 *
 * Many LTC/ADBMS devices detect wake on nCS edges while asleep.
 * Two low-high toggles with small delays are commonly recommended.
 */
void Wakeup_Sleep(void) {
    for (int i = 0; i < 2; i++) {
        ADBMS_nCS_Low();
        HAL_Delay(1);
        ADBMS_nCS_High();
        HAL_Delay(1);
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
void ADBMS_init(){
	Wakeup_Sleep();
	ADBMS_UNSNAP();
	ADBMS_startADCVoltage();
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
void ADBMS_startADCVoltage() {
	uint8_t cmd[4];
	uint16_t cmd_pec;

	AdcRD RD = RD_OFF;
	AdcCONT CONT = CONT_ON;
	AdcDCP DCP = DCP_OFF;
	AdcRSTF RSTF = RSTF_ON;
	AdcOW OW = OW_ALL_OFF;

	uint16_t commandWord = 0;

	// Pack command bits into the 16-bit container (MSB first on wire)
	commandWord |= (0    << 10);    // bit10 = 0        (fixed)
	commandWord |= (1    << 9);     // bit9  = 1        (fixed sync bit)
	commandWord |= (RD   << 8);     // bit8  = RD       (rate/redundancy)
	commandWord |= (CONT << 7);     // bit7  = CONT     (continuous mode)
	commandWord |= (1    << 6);     // bit6  = 1        (fixed)
	commandWord |= (1    << 5);     // bit5  = 1        (fixed)
	commandWord |= (DCP  << 4);     // bit4  = DCP      (balance during conv)
	commandWord |= (0    << 3);     // bit3  = 0        (fixed)
	commandWord |= (RSTF << 2);     // bit2  = RSTF     (reset digital filter)
	commandWord |= (OW & 0x03);     // bit1-0 = OW[1:0] (open-wire mode)

	// Split command word into two bytes (big-endian: high byte first)
	uint8_t firstCmdByte  = (uint8_t)(commandWord >> 8);
	uint8_t secondCmdByte = (uint8_t)(commandWord & 0xFF);

	// Construct full command frame: [CMD_H][CMD_L][PEC15_H][PEC15_L]
	cmd[0] = firstCmdByte;
	cmd[1] = secondCmdByte;
	cmd_pec = ADBMS_calcPec15(cmd, 2);   // PEC over the 2 command bytes
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	// Ensure the isoSPI port is awake before issuing the command
	isoSPI_Idle_to_Ready();

	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) cmd, 4, 100);
	ADBMS_nCS_High();
}

/**
 * @brief Issue SNAP command to latch a coherent “snapshot” of measurement registers.
 *
 * SNAP freezes a consistent set of measurement results so you can read them across
 * multiple RDCV/RDAUX/etc. transactions without internal updates racing your reads.
 * Typical flow: SNAP -> read all needed registers -> UNSNAP when done.
 */
void ADBMS_SNAP(){
	uint8_t  cmd[4];
	uint16_t cmd_pec;

	cmd[0] = SNAP[0];       // Command high byte
	cmd[1] = SNAP[1];	    // Command low byte
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready(); // Make sure link is awake

	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);
	ADBMS_nCS_High();
}

/**
 * @brief Issue UNSNAP command to resume live register updates after a SNAP.
 *
 * Call this once you’ve finished reading all latched measurement pages.
 */
void ADBMS_UNSNAP(){
	uint8_t  cmd[4];
	uint16_t cmd_pec;

	cmd[0] = UNSNAP[0];       // Command high byte
	cmd[1] = UNSNAP[1];	      // Command low byte
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready();   // Ensure link is awake

	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);
	ADBMS_nCS_High();
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
LTC_SPI_StatusTypeDef ADBMS_getAVGCellVoltages(ModuleData *mod) {
	LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
	HAL_StatusTypeDef hal_ret;
	const uint8_t  RX_BYTES_PER_IC = DATA_LEN + PEC_LEN;      // 6 data + 2 PEC per IC
	const uint16_t RX_LEN = (uint16_t)(RX_BYTES_PER_IC * NUM_MOD);
	uint8_t rx_buffer[RX_LEN];
	uint8_t  cmd[4];
	uint16_t cmd_pec;

	ADBMS_SNAP(); // Latch a consistent dataset across all modules

	// We have 14 cells per module; ADBMS_SERIES_GROUPS_PER_RDCV indicates how many cells per RDAC page.
	for (uint8_t regIndex = 0; regIndex < ((NUM_CELL_PER_MOD + ADBMS_SERIES_GROUPS_PER_RDCV - 1)
			/ ADBMS_SERIES_GROUPS_PER_RDCV); regIndex++) {

		// Build RDAC command for the current voltage-register page
		cmd[0] = (0xFF & (ADBMS_CMD_RDAC[regIndex] >> 8));
		cmd[1] = (0xFF & (ADBMS_CMD_RDAC[regIndex]));
		cmd_pec = ADBMS_calcPec15(cmd, 2);
		cmd[2] = (uint8_t) (cmd_pec >> 8);
		cmd[3] = (uint8_t) (cmd_pec);

		isoSPI_Idle_to_Ready(); // Ensure link is up before transaction

		ADBMS_nCS_Low();

		// Transmit the RDAC command
	    hal_ret = HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);
	    if (hal_ret != HAL_OK) {
	        ret |= (1U << (hal_ret + LTC_SPI_TX_BIT_OFFSET)); // Encode HAL error into return flags
	        ADBMS_UNSNAP();
	        ADBMS_nCS_High();
	        return ret;
	    }

	    // Receive one page from every device in the chain (concatenated)
	    hal_ret = HAL_SPI_Receive(&hspi1, rx_buffer, RX_LEN, 100);
	    if (hal_ret != HAL_OK) {
	        ret |= (1U << (hal_ret + LTC_SPI_RX_BIT_OFFSET));
	        ADBMS_UNSNAP();
	        ADBMS_nCS_High();
	        return ret;
	    }

	    ADBMS_nCS_High();

	    // Validate and unpack each device’s 6-byte data + 2-byte PEC10
	    for (uint8_t devIndex = 0; devIndex < NUM_MOD; devIndex++) {
			uint16_t offset = (uint16_t)(devIndex * RX_BYTES_PER_IC);

			uint8_t *voltData    = &rx_buffer[offset];            // 6 data bytes
			uint8_t *voltDataPec = &rx_buffer[offset + DATA_LEN]; // 2 PEC bytes

			// Check RX PEC10 for data integrity
			bool pec10Check = ADBMS_checkRxPec(voltData, DATA_LEN, voltDataPec);
			if (!pec10Check) {
				printf("M%d failed\n", devIndex + 1);
				continue; // Skip this device’s page if PEC fails
			}

			// Each page provides ADBMS_SERIES_GROUPS_PER_RDCV cells (2 bytes per cell)
			for (uint8_t dataIndex = 0; dataIndex < ADBMS_SERIES_GROUPS_PER_RDCV; dataIndex++) {
				uint8_t cellInMod = (uint8_t)(regIndex * ADBMS_SERIES_GROUPS_PER_RDCV + dataIndex);
				if (cellInMod >= NUM_CELL_PER_MOD) break; // Last page may be partially filled (e.g., 14 cells)

				// Device transmits LSB then MSB; re-pack into a 16-bit sample
				uint8_t lo = voltData[2 * dataIndex + 0];
				uint8_t hi = voltData[2 * dataIndex + 1];

				uint16_t ILOVEBMS = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);  //pack raw data to uint16_t

				// 0x8000 is a device “cleared/invalid” code; store 0xFFFF as a sentinel for “no data”
				if (ILOVEBMS == 0x8000u) { mod[devIndex].cell_volt[cellInMod] = 0xFFFF; }
				else {
					// Convert raw to mV: mv = 1500 mV + raw * 0.150 mV/LSB
                    // (Adjust this formula to the exact datasheet scaling for your part/mode.)
				    uint32_t mv = 1500u + (uint32_t)ILOVEBMS * 150u;             // in 0.01 mV units => here stored in mV
				    if (mv > 65535u) mv = 65535u;                                // Clamp to 16-bit storage field
				    mod[devIndex].cell_volt[cellInMod] = (uint16_t)mv;           //store in mV
				}
			}
		}
	}

//	for (int i = 0; i < NUM_MOD; i++){
//		for(int j = 0; j < NUM_CELL_PER_MOD; j++){
//			printf("Cell Voltage\n M%d:%d\n", (i + 1), mod->cell_volt[i][j]);
//		}
//	}

	ADBMS_UNSNAP();

	return ret;
}

/**
 * 	write command to all pwm registers. This setup only allows to use 4b'1111 (HIGH) or 4b'0000 (LOW). 
 * @param total_ic		total count of ic (daisy chain)
 * @param pwm			A two dimensional array of the configuration data that will be written
 */
void LTC_writePWM(uint8_t total_ic, uint8_t pwm) {
	// NOTE currently chaging this method to only assign a specific PWM to all registers

	// TODO change it back to relying on @param pwm for duty cycle assignment. 

	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4 + (8 * total_ic);
	uint16_t pwm_pec;
	uint16_t cmd_pec;
	uint8_t cmd_index; // command counter

	// init bits
	wrpwm_buffer[0] = 0x00;
	wrpwm_buffer[1] = 0x20;
	cmd_pec = ADBMS_calcPec15(wrpwm_buffer, 2);
	wrpwm_buffer[2] = (uint8_t) (cmd_pec >> 8);
	wrpwm_buffer[3] = (uint8_t) (cmd_pec);

	cmd_index = 4;				// Command bits
	for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--) // executes for each ltc6811 in daisy chain, this loops starts with
			{
		// the last IC on the stack. The first configuration written is
		// received by the last IC in the daisy chain

		for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG;
				current_byte++) // executes for each of the 6 bytes in the CFGR register
				{
			// current_byte is the byte counter

			wrpwm_buffer[cmd_index] = pwm; //adding the pwm data to the array to be sent
			cmd_index = cmd_index + 1;
		}

		pwm_pec = (uint16_t) ADBMS_calcPec15(&pwm, BYTES_IN_REG); // calculating the PEC for each ICs configuration register data
		wrpwm_buffer[cmd_index] = (uint8_t) (pwm_pec >> 8);
		wrpwm_buffer[cmd_index + 1] = (uint8_t) pwm_pec;
		cmd_index = cmd_index + 2;
	}

	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) wrpwm_buffer, CMD_LEN, 100);
	ADBMS_nCS_High();
}

void LTC_writeCFG(uint8_t total_ic, //The number of ICs being written to
		uint8_t config[][6] //A two dimensional array of the configuration data that will be written
		) {
	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4 + (8 * total_ic);
	uint16_t cfg_pec;
	uint8_t cmd_index; //command counter

	wrcfg_buffer[0] = 0x00;
	wrcfg_buffer[1] = 0x01;
	wrcfg_buffer[2] = 0x3d;
	wrcfg_buffer[3] = 0x6e;

	cmd_index = 4;
	// executes for each ltc6811 in daisy chain, this loops starts with
	for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--) {
		// the last IC on the stack. The first configuration written is
		// received by the last IC in the daisy chain

		// executes for each of the 6 bytes in the CFGR register
		for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG;
				current_byte++) {
			// current_byte is the byte counter

			wrcfg_buffer[cmd_index] = config[current_ic - 1][current_byte]; //adding the config data to the array to be sent
			cmd_index = cmd_index + 1;
		}

		cfg_pec = (uint16_t) ADBMS_calcPec15(&config[current_ic - 1][0], BYTES_IN_REG); // calculating the PEC for each ICs configuration register data
		wrcfg_buffer[cmd_index] = (uint8_t) (cfg_pec >> 8);
		wrcfg_buffer[cmd_index + 1] = (uint8_t) cfg_pec;
		cmd_index = cmd_index + 2;
	}

	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) wrcfg_buffer, CMD_LEN, 100);
	ADBMS_nCS_High();
}

/**
 *
 * @param total_ic	The number of ICs being written to
 * @param comm[6]	A two dimensional array of the comm data that will be written
 */
void LTC_SPI_writeCommunicationSetting(uint8_t total_ic, uint8_t comm[6]) {
	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4 + (8 * total_ic);
	uint16_t comm_pec;
	uint16_t cmd_pec;
	uint8_t cmd_index; // command counter

	wrcomm_buffer[0] = 0x07;
	wrcomm_buffer[1] = 0x21;
	cmd_pec = ADBMS_calcPec15(wrcomm_buffer, 2);
	wrcomm_buffer[2] = (uint8_t) (cmd_pec >> 8);
	wrcomm_buffer[3] = (uint8_t) (cmd_pec);

	cmd_index = 4;
	for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--) // executes for each ltc6811 in daisy chain, this loops starts with
			{
		// the last IC on the stack. The first configuration written is
		// received by the last IC in the daisy chain

		for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG;
				current_byte++) // executes for each of the 6 bytes in the CFGR register
				{
			// current_byte is the byte counter
			wrcomm_buffer[cmd_index] = comm[current_byte]; // adding the config data to the array to be sent
			cmd_index = cmd_index + 1;
		}
		comm_pec = (uint16_t) ADBMS_calcPec15(&comm[0], BYTES_IN_REG); // calculating the PEC for each ICs configuration register data
		wrcomm_buffer[cmd_index] = (uint8_t) (comm_pec >> 8);
		wrcomm_buffer[cmd_index + 1] = (uint8_t) comm_pec;
		cmd_index = cmd_index + 2;
	}

	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) wrcomm_buffer, CMD_LEN, 100);
	ADBMS_nCS_High();
}

/**
 * Shifts data in COMM register out over ltc6811 SPI/I2C port
 */
void LTC_SPI_requestData(uint8_t len) {

	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[0] = 0x07;
	cmd[1] = 0x23;
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) cmd, 4, 100);
	for (int i = 0; i < len * 3; i++) {
		HAL_SPI_Transmit(&hspi1, (uint8_t*) 0xFF, 1, 100);
	}
	ADBMS_nCS_High();
}

LTC_SPI_StatusTypeDef LTC_readGPIOs(uint16_t *read_auxiliary) {
	LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
	LTC_SPI_StatusTypeDef hal_ret;
	const uint8_t ARR_SIZE_REG = NUM_MOD * REG_LEN;
	uint8_t read_auxiliary_reg[ARR_SIZE_REG]; // Increased in size to handle multiple devices

	for (uint8_t i = 0;
			i < (NUM_AUX_SERIES_GROUPS / LTC_SERIES_GROUPS_PER_RDAUX); i++) {
		uint8_t cmd[4];
		uint16_t cmd_pec;

		cmd[0] = (0xFF & (LTC_CMD_AUXREG[i] >> 8)); // RDCV Register
		cmd[1] = (0xFF & (LTC_CMD_AUXREG[i]));		// RDCV Register
		cmd_pec = ADBMS_calcPec15(cmd, 2);
		cmd[2] = (uint8_t) (cmd_pec >> 8);
		cmd[3] = (uint8_t) (cmd_pec);

		isoSPI_Idle_to_Ready(); // Wake LTC up

		ADBMS_nCS_Low(); // Pull CS low

		hal_ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) cmd, 4, 100);
		if (hal_ret) {									// Non-zero means error
			ret |= (1 << (hal_ret + LTC_SPI_TX_BIT_OFFSET)); // TX error
		}

		hal_ret = HAL_SPI_Receive(&hspi1, (uint8_t*) read_auxiliary_reg,
				ARR_SIZE_REG, 100);
		if (hal_ret) {									// Non-zero means error
			ret |= (1 << (hal_ret + LTC_SPI_RX_BIT_OFFSET)); // RX error
		}

		ADBMS_nCS_High(); // Pull CS high

		// Process the received data
		for (uint8_t dev_idx = 0; dev_idx < NUM_MOD; dev_idx++) {
			// Assuming data format is [cell voltage, cell voltage, ..., PEC, PEC]
			// PEC for each device is the last two bytes of its data segment
			uint8_t *data_ptr = &read_auxiliary_reg[dev_idx * REG_LEN];

			memcpy(
					&read_auxiliary[dev_idx * NUM_AUX_SERIES_GROUPS
							+ i * LTC_SERIES_GROUPS_PER_RDAUX], data_ptr,
					REG_LEN - 2);
		}

	}

	return ret;
}



void LTC_startADC_GPIO(uint8_t MD, // ADC Mode
		uint8_t CHG // GPIO Channels to be measured)
		) {
	uint8_t cmd[4];
	uint16_t cmd_pec;
	uint8_t md_bits;

	md_bits = (MD & 0x02) >> 1;
	cmd[0] = md_bits + 0x04;
	md_bits = (MD & 0x01) << 7;
	cmd[1] = md_bits + 0x60 + CHG;
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	/*
	 isoSPI_Idle_to_Ready (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
	 output_low(LTC6811_CS);
	 spi_write_array(4,cmd);
	 output_high(LTC6811_CS);
	 */
	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) cmd, 4, 100);
	ADBMS_nCS_High();
}

int32_t LTC_POLLADC() {
	uint32_t start_time = HAL_GetTick();
	uint8_t finished = 0;
	uint8_t adc_status = 0xFF;	//initialize adc status
	uint8_t dummy_tx = 0x00;	//we need to send this to give clock LTC6811 so it can send data
	uint8_t cmd[4];
	uint16_t cmd_pec;
	//get ready for the PLADC command
	cmd[0] = 0x07;
	cmd[1] = 0x14;
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
	//Send PLADC command to LTC6811
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) cmd, 4, 100);
	ADBMS_nCS_High();
	//Check if ADC is done
	while (((HAL_GetTick() - start_time) < 210000)) {  // timeout at 210ms
		ADBMS_nCS_Low();
		HAL_SPI_Transmit(&hspi1, &dummy_tx, 1, 100);  //Send dummy byte and get adc status
		HAL_SPI_Receive(&hspi1, &adc_status, 1, 100);
		ADBMS_nCS_High();
		if (adc_status != 0xFF) {  // if it's not 0xFF, finish adc
			finished = 1;
			break;
		} else {
			HAL_Delay(1);  //delay 1ms
		}
	    }

	ADBMS_nCS_High();
	return (finished ? (HAL_GetTick() - start_time) : 0);
}



/* Read and store raw cell voltages at uint8_t 2d pointer */
int Calc_Pack_Voltage(uint16_t *read_voltages) {
	int packvoltage = 0;
	for (int i = 0; i < NUM_MOD * NUM_CELL_PER_MOD; i++) {
		packvoltage += read_voltages[i];
	}
	return packvoltage;
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
LTC_SPI_StatusTypeDef ADBMS_ReadSID(uint8_t read_sid[][DATA_LEN]) {
    LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
    HAL_StatusTypeDef hal_ret;

    const uint8_t  RX_BYTES_PER_IC = DATA_LEN + PEC_LEN;      // 6 data + 2 PEC10 = 8 per device
    const uint16_t RX_LEN = (uint16_t)(RX_BYTES_PER_IC * NUM_MOD);

    uint8_t rx_buffer[RX_LEN];  // Concatenated receive buffer for the whole chain

    uint8_t  cmd[4];   // [CMD_H][CMD_L][PEC15_H][PEC15_L]
    uint16_t cmd_pec;

    // 1) Build command frame with PEC15 over the two command bytes
    cmd[0] = RDSID[0];
    cmd[1] = RDSID[1];
    cmd_pec = ADBMS_calcPec15(cmd, 2);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    // 2) Transmit command, then receive concatenated response from all modules
    isoSPI_Idle_to_Ready();

    ADBMS_nCS_Low();
    hal_ret = HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);
    if (hal_ret != HAL_OK) {
    	// Map HAL error code into our bitfield (TX error region)
        ret |= (1U << (hal_ret + LTC_SPI_TX_BIT_OFFSET));
        ADBMS_nCS_High();
        return ret;
    }

    hal_ret = HAL_SPI_Receive(&hspi1, rx_buffer, RX_LEN, 100);
    if (hal_ret != HAL_OK) {
    	// Map HAL error code into our bitfield (RX error region)
        ret |= (1U << (hal_ret + LTC_SPI_RX_BIT_OFFSET));
        ADBMS_nCS_High();
        return ret;
    }
    ADBMS_nCS_High();

    // 3) Validate and unpack each module's payload
    for (uint8_t devIndex = 0; devIndex < NUM_MOD; devIndex++) {
        uint16_t offset = (uint16_t)(devIndex * RX_BYTES_PER_IC);

        uint8_t *sid    = &rx_buffer[offset];            // 6-byte SID
        uint8_t *sidpec = &rx_buffer[offset + DATA_LEN]; // 2-byte PEC10 (packed with CC)

        // Verify 10-bit PEC computed over the 6 SID bytes (+ command counter if present)
        bool pec10Check = ADBMS_checkRxPec(sid, DATA_LEN, sidpec);
        if (!pec10Check) {
        	printf("M%d failed\n", devIndex + 1);
            continue; // Skip copying this module’s SID if PEC fails
        }
        memcpy(read_sid[devIndex], sid, DATA_LEN);

        // Pretty print the 6-byte SID for this module
        printf("SID M%u: %02X %02X %02X %02X %02X %02X\r\n",
                       (unsigned)(devIndex + 1),
                       sid[0], sid[1], sid[2], sid[3], sid[4], sid[5]);

    }

    return ret;
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
