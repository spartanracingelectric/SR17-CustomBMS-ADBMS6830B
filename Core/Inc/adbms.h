#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"
#include "spi.h"
#include "string.h"

#ifndef INC_6811_H_
#define INC_6811_H_

#endif /* INC_6811_H_ */

#define RDACA 0x0044
#define RDACB 0x0046
#define RDACC 0x0048
#define RDACD 0x004A
#define RDACE 0x0049
#define RDACF 0x004B

#define LTC_CMD_RDAUXA 0x000C
#define LTC_CMD_RDAUXB 0x000E

#define LTC_SPI_TX_BIT_OFFSET 0	// Num bits to shift RX status code
#define LTC_SPI_RX_BIT_OFFSET 4	// Num bits to shift RX status code
#define REG_LEN 8// number of bytes in the register + 2 bytes for the PEC
#define PEC_LEN 2
#define DATA_LEN 6
#define ADBMS_SERIES_GROUPS_PER_RDCV 3 // Number of cell voltage groups per 8 byte register
#define LTC_SERIES_GROUPS_PER_RDAUX 3
#define NUM_AUX_SERIES_GROUPS 6 // Number of series groups

//------------------------------------------------------------------------------
// RD: Redundant Measurement Enable
//   0 = C-ADC only
//   1 = Redundant measurement (C-ADC + S-ADC for safety comparison)
//------------------------------------------------------------------------------
typedef enum {
    RD_OFF = 0, // Redundancy disabled (C-ADC only)
    RD_ON  = 1  // Redundancy enabled (C-ADC and S-ADC both measure)
} AdcRD;

//------------------------------------------------------------------------------
// CONT: Continuous Conversion
//   0 = One-shot conversion (triggered by each ADCV command)
//   1 = Continuous conversion (C-ADC runs continuously after start)
//------------------------------------------------------------------------------
typedef enum {
    CONT_OFF = 0, // Single conversion only
    CONT_ON  = 1  // Continuous conversion mode
} AdcCONT;

//------------------------------------------------------------------------------
// DCP: Discharge Permit During Measurement
//   0 = Disable cell discharge during measurement (pause balancing)
//   1 = Permit cell discharge during measurement (keep balancing active)
//   *Effective mainly in S-ADC related modes; has no effect in C-ADC continuous mode
//------------------------------------------------------------------------------
typedef enum {
    DCP_OFF = 0, // Pause PWM discharge during conversion
    DCP_ON  = 1  // Allow PWM discharge during conversion
} AdcDCP;

//------------------------------------------------------------------------------
// RSTF: Reset IIR Filter
//   0 = Keep current IIR filter state
//   1 = Reset IIR filter (normally set only on the first ADCV command)
//------------------------------------------------------------------------------
typedef enum {
    RSTF_OFF = 0, // Do not reset the filter
    RSTF_ON  = 1  // Reset the filter (first command only)
} AdcRSTF;

//------------------------------------------------------------------------------
// OW[1:0]: Open-Wire Test Mode (cell connection diagnostics)
//   00 = Open-wire test disabled
//   01 = Even cells only
//   10 = Odd cells only
//   11 = All cells
//------------------------------------------------------------------------------
typedef enum {
    OW_ALL_OFF = 0b00, // No open-wire detection
    OW_EVEN_ON = 0b01, // Check even-numbered cells
    OW_ODD_ON  = 0b10, // Check odd-numbered cells
    OW_ALL_ON  = 0b11  // Check all cells
} AdcOW;

typedef enum {
	LTC_SPI_OK = 0x00U, //0b00000000
	LTC_SPI_TX_ERROR = 0x02U, //0b00000010
	LTC_SPI_TX_BUSY = 0x04U, //0b00000100
	LTC_SPI_TX_TIMEOUT = 0x08U, //0b00001000
	LTC_SPI_RX_ERROR = 0x20U, //0b00100000
	LTC_SPI_RX_BUSY = 0x40U, //0b01000000
	LTC_SPI_RX_TIMEOUT = 0x80U	 //0b10000000
} LTC_SPI_StatusTypeDef;

extern uint8_t wrpwm_buffer[4 + (8 * NUM_MOD)];
extern uint8_t wrcfg_buffer[4 + (8 * NUM_MOD)];
extern uint8_t wrcomm_buffer[4 + (8 * NUM_MOD)];

void isoSPI_Idle_to_Ready(void);

void Wakeup_Sleep(void);

void ADBMS_init();

void ADBMS_startADCVoltage();

void ADBMS_UNSNAP();

void ADBMS_UNSNAP();

LTC_SPI_StatusTypeDef ADBMS_getAVGCellVoltages(ModuleData *mod);

/* write to PWM register to control balancing functionality */
void LTC_writePWM(uint8_t total_ic, uint8_t pwm);

void LTC_writeCFG(uint8_t total_ic, //The number of ICs being written to
		uint8_t config[][6] //A two dimensional array of the configuration data that will be written
		);

void LTC_SPI_writeCommunicationSetting(uint8_t total_ic, //The number of ICs being written to
		uint8_t comm[6] //A two dimensional array of the comm data that will be written
		);

void LTC_SPI_requestData(uint8_t len);

LTC_SPI_StatusTypeDef LTC_readGPIOs(uint16_t *read_auxiliary);

void LTC_startADC_GPIO(uint8_t MD, //ADC Mode
		uint8_t CHG //GPIO Channels to be measured)
		);

int32_t LTC_POLLADC();

int Calc_Pack_Voltage(uint16_t *read_voltages);

LTC_SPI_StatusTypeDef ADBMS_ReadSID(uint8_t read_sid[][DATA_LEN]);

uint16_t ADBMS_calcPec15(uint8_t *data, uint8_t len);

uint16_t ADBMS_calcPec10(uint8_t *pDataBuf, int nLength, uint8_t *commandCounter);

bool ADBMS_checkRxPec(const uint8_t *rxBuffer, int len, const uint8_t pec[2]);

