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

/* Wake LTC up from IDLE state into READY state */
void isoSPI_Idle_to_Ready(void) {
	uint8_t hex_ff = 0xFF;
	ADBMS_nCS_Low();							   // Pull CS low
	HAL_SPI_Transmit(&hspi1, &hex_ff, 1, 100);     // Send byte 0xFF to wake LTC up
	ADBMS_nCS_High();							   // Pull CS high
	HAL_Delay(1);
}

// wake up sleep
void Wakeup_Sleep(void) {
    for (int i = 0; i < 2; i++) {
        ADBMS_nCS_Low();
        HAL_Delay(1);
        ADBMS_nCS_High();
        HAL_Delay(1);
    }
}

void ADBMS_init(){
	Wakeup_Sleep();
	ADBMS_UNSNAP();
	ADBMS_startADCVoltage();
}

/*
 Starts cell voltage conversion
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

	//pack the command bit's in the commandWord
	commandWord |= (0    << 10);    // bit10 = 0
	commandWord |= (1    << 9);     // bit9  = 1
	commandWord |= (RD   << 8);     // bit8  = RD
	commandWord |= (CONT << 7);     // bit7  = CONT
	commandWord |= (1    << 6);     // bit6  = 1
	commandWord |= (1    << 5);     // bit5  = 1
	commandWord |= (DCP  << 4);     // bit4  = DCP
	commandWord |= (0    << 3);     // bit3  = 0
	commandWord |= (RSTF << 2);     // bit2  = RSTF
	commandWord |= (OW & 0x03);     // bit1-0 = OW[1:0]

	//split into bytes and pack it to cmd
	uint8_t firstCmdByte  = (uint8_t)(commandWord >> 8);
	uint8_t secondCmdByte = (uint8_t)(commandWord & 0xFF);

	cmd[0] = firstCmdByte;
	cmd[1] = secondCmdByte;
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready(); // This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
	ADBMS_nCS_Low();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) cmd, 4, 100);
	ADBMS_nCS_High();
}

void ADBMS_SNAP(){
	uint8_t  cmd[4];
	uint16_t cmd_pec;

	cmd[0] = SNAP[0]; // RDAC Register
	cmd[1] = SNAP[1];	    // RDAC Register
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready(); // Wake LTC up

	ADBMS_nCS_Low(); // Pull CS low

	HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);

	ADBMS_nCS_High();
}

void ADBMS_UNSNAP(){
	uint8_t  cmd[4];
	uint16_t cmd_pec;

	cmd[0] = UNSNAP[0]; // RDAC Register
	cmd[1] = UNSNAP[1];	    // RDAC Register
	cmd_pec = ADBMS_calcPec15(cmd, 2);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);

	isoSPI_Idle_to_Ready(); // Wake LTC up

	ADBMS_nCS_Low(); // Pull CS low

	HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);

	ADBMS_nCS_High();
}

/* Read and store raw cell voltages at uint8_t 2d pointer */
LTC_SPI_StatusTypeDef ADBMS_getAVGCellVoltages(ModuleData *mod) {
	LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
	HAL_StatusTypeDef hal_ret;
	const uint8_t  RX_BYTES_PER_IC = DATA_LEN + PEC_LEN;      // 6+2=8
	const uint16_t RX_LEN = (uint16_t)(RX_BYTES_PER_IC * NUM_MOD);
	uint8_t rx_buffer[RX_LEN];

	uint8_t  cmd[4];
	uint16_t cmd_pec;

	ADBMS_SNAP();

	for (uint8_t regIndex = 0; regIndex < ((NUM_CELL_PER_MOD + ADBMS_SERIES_GROUPS_PER_RDCV - 1)
			/ ADBMS_SERIES_GROUPS_PER_RDCV); regIndex++) { //We have 14 cells, so we need to read 5 registers

		cmd[0] = (0xFF & (ADBMS_CMD_RDAC[regIndex] >> 8)); // RDAC Register
		cmd[1] = (0xFF & (ADBMS_CMD_RDAC[regIndex]));	    // RDAC Register
		cmd_pec = ADBMS_calcPec15(cmd, 2);
		cmd[2] = (uint8_t) (cmd_pec >> 8);
		cmd[3] = (uint8_t) (cmd_pec);

		isoSPI_Idle_to_Ready(); // Wake LTC up

		ADBMS_nCS_Low(); // Pull CS low

	    hal_ret = HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);
	    if (hal_ret != HAL_OK) {
	        ret |= (1U << (hal_ret + LTC_SPI_TX_BIT_OFFSET));
	        ADBMS_UNSNAP();
	        ADBMS_nCS_High();
	        return ret;
	    }

	    hal_ret = HAL_SPI_Receive(&hspi1, rx_buffer, RX_LEN, 100);
	    if (hal_ret != HAL_OK) {
	        ret |= (1U << (hal_ret + LTC_SPI_RX_BIT_OFFSET));
	        ADBMS_UNSNAP();
	        ADBMS_nCS_High();
	        return ret;
	    }

	    ADBMS_nCS_High();// Pull CS high

	    //check pec using pec10 for rx
	    for (uint8_t devIndex = 0; devIndex < NUM_MOD; devIndex++) {
			uint16_t offset = (uint16_t)(devIndex * RX_BYTES_PER_IC);

			uint8_t *voltData    = &rx_buffer[offset];
			uint8_t *voltDataPec = &rx_buffer[offset + DATA_LEN];

			bool pec10Check = ADBMS_checkRxPec(voltData, DATA_LEN, voltDataPec);
			if (!pec10Check) {
				printf("M%d failed\n", devIndex + 1);
				continue; // if pec is wrong ignore that chip
			}

			//put data into array
			for (uint8_t dataIndex = 0; dataIndex < ADBMS_SERIES_GROUPS_PER_RDCV; dataIndex++) {
				uint8_t cellInMod = (uint8_t)(regIndex * ADBMS_SERIES_GROUPS_PER_RDCV + dataIndex);
				if (cellInMod >= NUM_CELL_PER_MOD) break;  //cell number is 14, so last register only have 2 data

				// if LSB and MSB is opposite, flip this two
				uint8_t lo = voltData[2 * dataIndex + 0];
				uint8_t hi = voltData[2 * dataIndex + 1];

				//convert the raw data to mV
				uint16_t ILOVEBMS = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);  //pack raw data to uint16_t
				if (ILOVEBMS == 0x8000u) { mod[devIndex].cell_volt[cellInMod] = 0xFFFF; } //ADBMS will reset register to 8000 after clear command or power cycle
				else {
				    uint32_t mv = 1500u + (uint32_t)ILOVEBMS * 150u;             // 1.5V + 0.150mV/LSB
				    if (mv > 65535u) mv = 65535u;                                //
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


LTC_SPI_StatusTypeDef ADBMS_ReadSID(uint8_t read_sid[][DATA_LEN]) {
    LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
    HAL_StatusTypeDef hal_ret;

    const uint8_t  RX_BYTES_PER_IC    = DATA_LEN + PEC_LEN;      // 6+2=8
    const uint16_t RX_LEN          = (uint16_t)(RX_BYTES_PER_IC * NUM_MOD);

    uint8_t rx_buffer[RX_LEN];

    uint8_t  cmd[4];
    uint16_t cmd_pec;

    cmd[0] = RDSID[0];
    cmd[1] = RDSID[1];
    cmd_pec = ADBMS_calcPec15(cmd, 2);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    isoSPI_Idle_to_Ready();

    ADBMS_nCS_Low();
    hal_ret = HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 100);
    if (hal_ret != HAL_OK) {
        ret |= (1U << (hal_ret + LTC_SPI_TX_BIT_OFFSET));
        ADBMS_nCS_High();
        return ret;
    }

    hal_ret = HAL_SPI_Receive(&hspi1, rx_buffer, RX_LEN, 100);
    if (hal_ret != HAL_OK) {
        ret |= (1U << (hal_ret + LTC_SPI_RX_BIT_OFFSET));
        ADBMS_nCS_High();
        return ret;
    }
    ADBMS_nCS_High();


    for (uint8_t devIndex = 0; devIndex < NUM_MOD; devIndex++) {
        uint16_t offset = (uint16_t)(devIndex * RX_BYTES_PER_IC);

        uint8_t *sid    = &rx_buffer[offset];
        uint8_t *sidpec = &rx_buffer[offset + DATA_LEN];

        bool pec10Check = ADBMS_checkRxPec(sid, DATA_LEN, sidpec);
        if (!pec10Check) {
        	printf("M%d failed\n", devIndex + 1);
            continue; // if pec is wrong ignore that chip
        }
        memcpy(read_sid[devIndex], sid, DATA_LEN);
        printf("SID M%u: %02X %02X %02X %02X %02X %02X\r\n",
                       (unsigned)(devIndex + 1),
                       sid[0], sid[1], sid[2], sid[3], sid[4], sid[5]);

    }

    return ret;
}

/**
 * error calculation and handling for poor command use. 
 * @param 	len		Number of bytes that will be used to calculate a PEC
 * @param	data	Array of data that will be used to calculate a PEC
 */

uint16_t ADBMS_calcPec15(uint8_t *data, uint8_t len)
{
    uint16_t remainder, addr;
    remainder = 16; /* initialize the PEC */
    for (uint8_t i = 0; i<len; i++) /* loops for each byte in data array */
    {
        addr = (((remainder>>7)^data[i])&0xff);/* calculate PEC table address */
        remainder = ((remainder<<8)^Crc15Table[addr]);
    }
    return(remainder*2);/* The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}


uint16_t ADBMS_calcPec10(uint8_t *pDataBuf, int nLength, uint8_t *commandCounter)
{
    uint16_t nRemainder = 16u; /* PEC_SEED */
    /* x10 + x7 + x3 + x2 + x + 1 <- the CRC10 polynomial 100 1000 1111 */
    uint16_t nPolynomial = 0x8Fu;
    uint8_t nByteIndex, nBitIndex;
    uint16_t nTableAddr;

    for (nByteIndex = 0u; nByteIndex < nLength; ++nByteIndex)
    {
        /* calculate PEC table address */
        nTableAddr = (uint16_t)(((uint16_t)(nRemainder >> 2) ^ (uint8_t)pDataBuf[nByteIndex]) & (uint8_t)0xff);
        nRemainder = (uint16_t)(((uint16_t)(nRemainder << 8)) ^ crc10Table[nTableAddr]);
    }
    /* If array is from received buffer add command counter to crc calculation */
    if (commandCounter != NULL)
    {
        nRemainder ^= (uint16_t)(*commandCounter << 4u);
    }
    /* Perform modulo-2 division, a bit at a time */
    for (nBitIndex = 6u; nBitIndex > 0u; --nBitIndex)
    {
        /* Try to divide the current data bit */
        if ((nRemainder & 0x200u) > 0u)
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
            nRemainder = (uint16_t)(nRemainder ^ nPolynomial);
        }
        else
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
        }
    }
    return ((uint16_t)(nRemainder & 0x3FFu));
}


bool ADBMS_checkRxPec(const uint8_t *rxBuffer, int len, const uint8_t pec[2])
{
    uint8_t  cc   = (uint8_t)((pec[0] >> 2) & 0x3F); //get cc(6bits) from the first 6 bits of pec
    uint16_t dpec = (uint16_t)(((pec[0] & 0x03) << 8) | pec[1]);  // 10bit

    uint16_t calc = (uint16_t)(ADBMS_calcPec10((uint8_t*)rxBuffer, len, &cc) & 0x03FF);
    return ((dpec & 0x03FF) == calc);
}
