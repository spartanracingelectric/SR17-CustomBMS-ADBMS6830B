/**
 * @file    adbms6811.h
 * @brief   Public interface for ADBMS/LTC6811-like battery monitor SPI control.
 *
 * This header collects:
 *  - Command codes (RDCV/RDAUX variants, etc.)
 *  - Common constants (frame lengths, PEC sizes)
 *  - ADC control field enums (RD, CONT, DCP, RSTF, OW)
 *  - Status bitfield for SPI/HAL error reporting
 *  - Public function prototypes for init, conversion, register access, and PEC helpers
 *
 * Conventions:
 *  - All multi-byte command frames are big-endian on the wire: [CMD_H][CMD_L][PEC_H][PEC_L].
 *  - Device data blocks are typically 6 data bytes + 2-byte PEC10 per IC per read.
 *  - PEC15 protects outbound command bytes; PEC10 protects inbound data.
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"
#include "spi.h"
#include "string.h"

#ifndef ADBMS6830_H_
#define ADBMS6830_H_

#endif /* INC_6811_H_ */

/* ===== Register Read Command Codes (cell voltage pages) ======================
 * Each RDACx reads one page ("group") of series cell voltages.
 * Exact mapping (which cells appear in A/B/C/…) depends on the device family/datasheet.
 * These are 16-bit opcodes that will be followed by a PEC15 when transmitted.
 */
#define RDACA 0x0044
#define RDACB 0x0046
#define RDACC 0x0048
#define RDACD 0x004A
#define RDACE 0x0049
#define RDACF 0x004B

#define RDSID 0x002C


#define SNAP 0x002D
#define UNSNAP 0x002F

#define CLRCELL 0x0711
#define CLRAUX 0x712
#define CLRFC 0x0714
#define CLOVUV 0x0715
#define CLRSPIN 0x0716
#define CLRFLAG 0x0717


#define VUV_FROM_VOLTAGE(v)   ((uint16_t)(((v) - 1.5f) / (16.0f * 150e-6f)))
#define VOV_FROM_VOLTAGE(v)   ((uint16_t)(((v) - 1.5f) / (16.0f * 150e-6f)))

#define VUV   VUV_FROM_VOLTAGE(2.50f)
#define VOV   VOV_FROM_VOLTAGE(4.2f) 


/* ===== Auxiliary (GPIO/Ref) Register Read Command Codes =====================
 * Read auxiliary measurement pages (e.g., GPIO voltages, Vref, etc.).
 */
#define RDAUXA 0x0019
#define RDAUXB 0x001A
#define RDAUXC 0x001B
#define RDAUXD 0x001F

#define RDSTATA 0x0030
#define RDSTATB 0x0031
#define RDSTATC 0x0072
#define RDSTATD 0x0033
#define RDSTATE 0x0034


/* ===== Frame Sizes and Packing ==============================================
 * REG_LEN: 8 bytes per IC response frame: 6 data + 2-byte PEC10
 * PEC_LEN: number of PEC bytes in an IC data frame (2 bytes carry [CC|CRC10])
 * DATA_LEN: number of data bytes per IC per page (typically 6)
 * ADBMS_SERIES_GROUPS_PER_RDCV: cells per RDCV page (device-specific; 3 is common)
 * LTC_SERIES_GROUPS_PER_RDAUX: aux channels per RDAUX page (3 here)
 * NUM_AUX_SERIES_GROUPS: total number of aux channels across pages (6 here)
 */
#define REG_LEN 8
#define PEC_LEN 2
#define DATA_LEN 6
#define COMMAND_LENGTH 2
#define RX_BYTES_PER_IC (DATA_LEN + PEC_LEN)
#define RX_LEN (RX_BYTES_PER_IC * NUM_MOD)
// #define FRAME_LENGTH (COMMAND_LENGTH + PEC_LEN + (NUM_MOD * (DATA_LEN + PEC_LEN)))
#define CELLS_PER_REGISTER 3
#define ADBMS_SERIES_GROUPS_PER_RDAC 3
#define ADBMS_SERIES_GROUPS_PER_RDAUX 3
#define REG_NUM_RDAUX 4
#define REG_NUM_RDSTAT 5
#define NUM_AUX_SERIES_GROUPS 6

#define WRCFGA 0x0001
#define WRCFGB 0x0024
#define RDCFGB 0x0026

/* ===== ADC Control Field Enums ==============================================
 * These map directly to bitfields inside the ADCV (start conversion) command.
 * See datasheet for exact bit positions; your .c packs them into a command word.
 */

//------------------------------------------------------------------------------
// RD: Redundant Measurement Enable
//   0 = C-ADC only
//   1 = Redundant measurement (C-ADC + S-ADC for safety comparison)
//------------------------------------------------------------------------------
typedef enum {
    REDUNDANT_MODE_OFF = 0, // Redundancy disabled (C-ADC only)
    REDUDANT_MODE_ON  = 1  // Redundancy enabled (C-ADC and S-ADC both measure)
} AdcRedundantMode;

//------------------------------------------------------------------------------
// CONT: Continuous Conversion
//   0 = One-shot conversion (triggered by each ADCV command)
//   1 = Continuous conversion (C-ADC runs continuously after start)
//------------------------------------------------------------------------------
typedef enum {
    CONTINUOUS_MODE_OFF = 0, // Single conversion only
    CONTINUOUS_MODE_ON  = 1  // Continuous conversion mode
} AdcContinuousMode;

//------------------------------------------------------------------------------
// DCP: Discharge Permit During Measurement
//   0 = Disable cell discharge during measurement (pause balancing)
//   1 = Permit cell discharge during measurement (keep balancing active)
//   *Effective mainly in S-ADC related modes; has no effect in C-ADC continuous mode
//------------------------------------------------------------------------------
typedef enum {
    DISCHARGE_MODE_OFF = 0, // Pause PWM discharge during conversion
    DISCHARGE_MODE_ON = 1  // Allow PWM discharge during conversion
} AdcDischargeMode;

//------------------------------------------------------------------------------
// RSTF: Reset IIR Filter
//   0 = Keep current IIR filter state
//   1 = Reset IIR filter (normally set only on the first ADCV command)
//------------------------------------------------------------------------------
typedef enum {
    FILTER_RESET_MODE_OFF = 0, // Do not reset the filter
    FILTER_RESET_MODE_ON  = 1  // Reset the filter (first command only)
} AdcFilterResetMode;

//------------------------------------------------------------------------------
// OW[1:0]: Open-Wire Test Mode (cell connection diagnostics)
//   00 = Open-wire test disabled
//   01 = Even cells only
//   10 = Odd cells only
//   11 = All cells
//------------------------------------------------------------------------------
typedef enum {
    OPEN_WIRE_MODE_ALL_OFF = 0b00, // No open-wire detection
    OPEN_WIRE_MODE_EVEN_ON = 0b01, // Check even-numbered cells
    OPEN_WIRE_MODE_ODD_ON  = 0b10, // Check odd-numbered cells
    OPEN_WIRE_MODE_ALL_ON  = 0b11  // Check all cells
} AdcOpenWireMode;

typedef enum {
	OW_OFF = 0, // Check open-wire for AUX
	OW_ON  = 1  // Check open-wire for AUX
} AUXOW;

typedef enum {
	PUP_OFF = 0, // Check open-wire for AUX
	PUP_ON  = 1  // Check open-wire for AUX
} AUXPUP;

typedef enum {
	CH4_ON  = 0, // Check open-wire for AUX
	CH4_OFF = 1  // Check open-wire for AUX
} AUXCH4;

typedef enum {
	CH3_ON  = 0, // Check open-wire for AUX
	CH3_OFF = 1  // Check open-wire for AUX
} AUXCH3;

typedef enum {
	CH2_ON  = 0, // Check open-wire for AUX
	CH2_OFF = 1  // Check open-wire for AUX
} AUXCH2;

typedef enum {
	CH1_ON  = 0, // Check open-wire for AUX
	CH1_OFF = 1  // Check open-wire for AUX
} AUXCH1;

typedef enum {
	CH0_ON  = 0, // Check open-wire for AUX
	CH0_OFF = 1  // Check open-wire for AUX
} AUXCH0;


/* ===== SPI Status Bitfield ===================================================
 * Compose these flags to reflect HAL TX/RX outcomes without throwing assertions.
 * Example: set (1U << (hal_ret + LTC_SPI_TX_BIT_OFFSET)) on TX failure.
 */


/* ===== External TX Buffers ===================================================
 * Staging buffers for multi-IC write commands. Each frame = 4 cmd/PEC bytes +
 * (8 bytes per IC) payload. They are defined in a .c and referenced here.
 */
extern uint8_t wrpwm_buffer[4 + (8 * NUM_MOD)];
extern uint8_t wrcfg_buffer[4 + (8 * NUM_MOD)];
extern uint8_t wrcomm_buffer[4 + (8 * NUM_MOD)];

/* ===== Public API: Link/Power State Helpers =================================
 * isoSPI_Idle_to_Ready(): send a dummy 0xFF while nCS is low to wake IDLE->READY.
 * Wakeup_Sleep(): toggle nCS to bring devices out of SLEEP (no clocks needed).
 */
void isoSPI_Idle_to_Ready();
void ADBMS_wakeUp();
void ADBMS_clearRegisters();

/* ===== Public API: High-Level Init/Control ==================================
 * ADBMS_init(): wake devices, UNSNAP to resume live updates, and start ADCV.
 * ADBMS_startADCVoltage(): build and send ADCV command (RD/CONT/DCP/RSTF/OW fields).
 * ADBMS_SNAP()/UNSNAP(): latch a coherent dataset across pages / resume updates.
 */
void ADBMS_init();
void ADBMS_startCellVoltageConversions(AdcRedundantMode redundantMode, AdcContinuousMode continuousMode, AdcDischargeMode dischargeMode, AdcFilterResetMode filterResetMode, AdcOpenWireMode openWireMode);
void ADBMS_snap();
void ADBMS_unsnap();

/* ===== Public API: Measurement and Register Access ===========================
 * ADBMS_getCellVoltages(): read all RDCV pages from every IC (with PEC check),
 *                             convert raw counts to mV, and store into ModuleData.
 */
void ADBMS_getCellVoltages(ModuleData *moduleData);
void ADBMS_writeConfigurationRegisterB(BalanceStatus *blst);
void ADBMS_parseConfigurationRegisterB(uint8_t data[DATA_LEN], ConfigurationRegisterB *configB);
void ADBMS_readConfigurationRegisterB(ConfigurationRegisterB *configB);
void ADBMS_ReadSID(ModuleData *mod);
void ADBMS_sendCommand(uint16_t command);
void ADBMS_receiveData(uint8_t rxBuffer[NUM_MOD][DATA_LEN + PEC_LEN]);
void ADBMS_parseVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData);
LTC_SPI_StatusTypeDef ADBMS_getGPIOData(ModuleData *mod);
LTC_SPI_StatusTypeDef ADBMS_getVref2(ModuleData *mod);

/* ===== Public API: PEC Helpers ==============================================
 * ADBMS_calcPec15(): compute CRC15 (PEC15) for command bytes (returns LSB=0).
 * ADBMS_calcPec10(): compute CRC10 (PEC10) for data bytes; can fold 6-bit CC.
 * ADBMS_checkRxPec(): extract CC+CRC10 from 2-byte field and verify against data.
 */
uint16_t ADBMS_calcPec15(uint8_t *data, uint8_t len);
uint16_t ADBMS_calcPec10(uint8_t *pDataBuf, int nLength, uint8_t *commandCounter);
bool ADBMS_checkRxPec(const uint8_t *rxBuffer, int len, const uint8_t pec[2]);

