/**
 * @file    adbms6811.h
 */
#ifndef ADBMS6830_H_
#define ADBMS6830_H_

#include "main.h"
#include "spi.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "module.h"
#include "balance.h"

// Voltage conversion base commands
#define ADCV 0x0260
#define ADSV 0x0168

// Auxiliary conversion base commands
#define ADAX 0x0410
#define ADAX2 0x0400

// Read cell voltage registers A-F commands
#define RDCVA 0x0004
#define RDCVB 0x0006
#define RDCVC 0x0008
#define RDCVD 0x000A
#define RDCVE 0x0009
#define RDCVF 0x000B
#define NUM_CELL_VOLTAGE_REGISTERS 6

// Read average cell voltage registers A-F commands
#define RDACA 0x0044
#define RDACB 0x0046
#define RDACC 0x0048
#define RDACD 0x004A
#define RDACE 0x0049
#define RDACF 0x004B
#define NUM_AVERAGE_CELL_VOLTAGE_REGISTERS 6

// Read redundant cell voltage registers A-F commands
#define RDSVA 0x0003
#define RDSVB 0x0005
#define RDSVC 0x0007
#define RDSVD 0x000D
#define RDSVE 0x000E
#define RDSVF 0x000F
#define NUM_REDUNDANT_CELL_VOLTAGE_REGISTERS 6

// Read auxiliary registers A-D commands
#define RDAUXA 0x0019
#define RDAUXB 0x001A
#define RDAUXC 0x001B
#define RDAUXD 0x001F
#define NUM_AUX_REGISTERS 4

// Read redundant auxiliary registers A-D commands
#define RDRAXA 0x001C
#define RDRAXB 0x001D
#define RDRAXC 0x001E
#define RDRAXD 0x0025
#define NUM_REDUNDANT_AUX_REGISTERS 4

// Read status registers A-E commands
#define RDSTATA 0x0030
#define RDSTATB 0x0031
#define RDSTATC 0x0072
#define RDSTATD 0x0033
#define RDSTATE 0x0034
#define NUM_STATUS_REGISTERS 5

// Read ADBMS IC serial ID
#define RDSID 0x002C

// Clear status registers commands
#define CLRCELL 0x0711
#define CLRAUX 0x712
#define CLRFC 0x0714
#define CLOVUV 0x0715
#define CLRSPIN 0x0716
#define CLRFLAG 0x0717
#define NUM_CLEAR_COMMANDS 6

// Snap and unsnap to freeze and unfreeze measurements registers
#define SNAP 0x002D
#define UNSNAP 0x002F

// Write to configuration reigster commands
#define WRCFGA 0x0001
#define WRCFGB 0x0024

// Read from configuration register commands
#define RDCFGA 0x0002
#define RDCFGB 0x0026

// PEC calculation macros
#define INITIAL_CRC_SEED 0x10
#define COMMAND_CRC_POLYNOMIAL 0x4599
#define DATA_CRC_POLYNOMIAL 0x08F

// Default voltage register value if no conversion command is called
#define DEFAULT_VOLTAGE_VALUE 0x8000

// Placeholder threshold for what voltage should drop to when doing open wire check
#define OPEN_WIRE_CHECK_VOLTAGE_MV 1000.0f

#define VUV_FROM_VOLTAGE(v) ((uint16_t)(((v) - 1.5f) / (16.0f * 150e-6f)))
#define VOV_FROM_VOLTAGE(v) ((uint16_t)(((v) - 1.5f) / (16.0f * 150e-6f)))
#define VUV VUV_FROM_VOLTAGE(2.50f)
#define VOV VOV_FROM_VOLTAGE(4.2f)

#define REG_LEN 8
#define PEC_LEN 2
#define DATA_LEN 6
#define COMMAND_LENGTH 2
#define RX_BYTES_PER_IC (DATA_LEN + PEC_LEN)
#define RX_LEN (RX_BYTES_PER_IC * NUM_MOD)

// #define FRAME_LENGTH (COMMAND_LENGTH + PEC_LEN + (NUM_MOD * (DATA_LEN + PEC_LEN)))
#define CELLS_PER_ADC_REGISTER 3
#define GPIOS_PER_AUX_REGISTER 3
#define NUMBER_OF_AUX_REGISTERS 4
#define GPIOS_PER_IC 10
#define REG_NUM_RDAUX 4
#define REG_NUM_RDSTAT 5
#define NUM_AUX_SERIES_GROUPS 6

/* ===== ADC Control Field Enums ==============================================
 * These map directly to bitfields inside the ADCV (start conversion) command.
 * See datasheet for exact bit positions
 */

//------------------------------------------------------------------------------
// RD: Redundant Measurement Enable
//   0 = C-ADC only
//   1 = Redundant measurement (C-ADC + S-ADC for safety comparison)
//------------------------------------------------------------------------------
typedef enum
{
	REDUNDANT_MODE_OFF = 0, // Redundancy disabled (C-ADC only)
	REDUDANT_MODE_ON = 1    // Redundancy enabled (C-ADC and S-ADC both measure)
} AdcRedundantMode;

//------------------------------------------------------------------------------
// CONT: Continuous Conversion
//   0 = One-shot conversion (triggered by each ADCV command)
//   1 = Continuous conversion (C-ADC runs continuously after start)
//------------------------------------------------------------------------------
typedef enum
{
	CONTINUOUS_MODE_OFF = 0, // Single conversion only
	CONTINUOUS_MODE_ON = 1   // Continuous conversion mode
} AdcContinuousMode;

//------------------------------------------------------------------------------
// DCP: Discharge Permit During Measurement
//   0 = Disable cell discharge during measurement (pause balancing)
//   1 = Permit cell discharge during measurement (keep balancing active)
//   *Effective mainly in S-ADC related modes; has no effect in C-ADC continuous mode
//------------------------------------------------------------------------------
typedef enum
{
	DISCHARGE_MODE_OFF = 0, // Pause PWM discharge during conversion
	DISCHARGE_MODE_ON = 1   // Allow PWM discharge during conversion
} AdcDischargeMode;

//------------------------------------------------------------------------------
// RSTF: Reset IIR Filter
//   0 = Keep current IIR filter state
//   1 = Reset IIR filter (normally set only on the first ADCV command)
//------------------------------------------------------------------------------
typedef enum
{
	FILTER_RESET_MODE_OFF = 0, // Do not reset the filter
	FILTER_RESET_MODE_ON = 1   // Reset the filter (first command only)
} AdcFilterResetMode;

//------------------------------------------------------------------------------
// OW[1:0]: Open-Wire Test Mode (cell connection diagnostics)
//   00 = Open-wire test disabled
//   01 = Even cells only
//   10 = Odd cells only
//   11 = All cells
//------------------------------------------------------------------------------
typedef enum
{
	OPEN_WIRE_MODE_ALL_OFF = 0b00, // No open-wire detection
	OPEN_WIRE_MODE_EVEN_ON = 0b01, // Check even-numbered cells
	OPEN_WIRE_MODE_ODD_ON = 0b10,  // Check odd-numbered cells
	OPEN_WIRE_MODE_ALL_ON = 0b11   // Check all cells
} AdcOpenWireMode;

typedef enum
{
	OW_OFF = 0,
	OW_ON = 1
} AuxOpenWireMode;

typedef enum
{
	PUP_OFF = 0,
	PUP_ON = 1
} AuxPullUpPinMode;

typedef enum
{
	AUX_CHANNEL_ALL = 0x00, // Measure all channels
	AUX_CHANNEL_GPIO1 = 0x01,
	AUX_CHANNEL_GPIO2 = 0x02,
	AUX_CHANNEL_GPIO3 = 0x03,
	AUX_CHANNEL_GPIO4 = 0x04,
	AUX_CHANNEL_GPIO5 = 0x05,
	AUX_CHANNEL_GPIO6 = 0x06,
	AUX_CHANNEL_GPIO7 = 0x07,
	AUX_CHANNEL_GPIO8 = 0x08,
	AUX_CHANNEL_GPIO9 = 0x09,
	AUX_CHANNEL_GPIO10 = 0x0A,
	AUX_CHANNEL_VD = 0x0B,    // Supply voltage VD
	AUX_CHANNEL_VA = 0x0C,    // Supply voltage VA
	AUX_CHANNEL_VREF2 = 0x0D, // Secondary reference voltage
	AUX_CHANNEL_ITEMP = 0x0E, // Internal temperature
	AUX_CHANNEL_NONE = 0x1F   // Reserved / invalid
} AuxChannelSelect;

typedef enum
{
	DIAGNOSTIC_PHASE_REDUNDANT_START,
	DIAGNOSTIC_PHASE_REDUNDANT_RUNNING,
	DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_ODD,
	DIAGNOSTIC_PHASE_CELL_OPEN_WIRE_EVEN,
} DiagnosticPhase;

extern DiagnosticPhase diagnosticPhase;

void isoSPI_Idle_to_Ready();
void ADBMS_wakeUp();
void ADBMS_clearRegisters();

void ADBMS_init();

void ADBMS_startCellVoltageConversions(AdcRedundantMode redundantMode, AdcContinuousMode continuousMode, AdcDischargeMode dischargeMode, AdcFilterResetMode filterResetMode, AdcOpenWireMode openWireMode);
void ADBMS_startRedundantCellVoltageConversions(AdcContinuousMode continuousMode, AdcDischargeMode dischargeMode, AdcOpenWireMode openWireMode);

void ADBMS_getAverageCellVoltages(ModuleData *moduleData);
void ADBMS_getCellVoltages(ModuleData *moduleData);
void ADBMS_parseRedundantCellVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData);
void ADBMS_parseCellVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData);

void ADBMS_checkDiagnostics(ModuleData *moduleData);
void ADBMS_getRedundantFaultFlags(ModuleData *moduleData);
void ADBMS_parseRedundantFaultFlags(ModuleData *moduleData, uint8_t rxBuffer[NUM_MOD][REG_LEN]);

void ADBMS_snap();
void ADBMS_unsnap();

void ADBMS_writeConfigurationRegisterB(BalanceStatus *blst);
void ADBMS_parseConfigurationRegisterB(uint8_t data[DATA_LEN], ConfigurationRegisterB *configB);
void ADBMS_readConfigurationRegisterB(ConfigurationRegisterB *configB);

void ADBMS_readSID(ModuleData *mod);

void ADBMS_sendCommand(uint16_t command);
void ADBMS_receiveData(uint8_t rxBuffer[NUM_MOD][DATA_LEN + PEC_LEN]);

void ADBMS_startAuxConversions(AuxOpenWireMode openWireMode, AuxPullUpPinMode pullUpPinMode, AuxChannelSelect channelSelect);
void ADBMS_getGpioVoltages(ModuleData *moduleData);
void ADBMS_parseGpioVoltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], uint8_t registerIndex, ModuleData *moduleData);
void ADBMS_getVref2(ModuleData *mod);
void ADBMS_parseVref2Voltages(uint8_t rxBuffer[NUM_MOD][REG_LEN], ModuleData *moduleData);

uint16_t ADBMS_calculateCommandPec(uint8_t *data, int length);
uint16_t ADBMS_calculateDataPec(uint8_t *data, int length, uint8_t commandCounter);
bool ADBMS_checkDataPec(uint8_t *rxBuffer, uint16_t length, uint8_t pec[2]);
void ADBMS_generateCrc15Table(void);
void ADBMS_generateCrc10Table8Bit(void);
void ADBMS_generateCrc10Table6Bit(void);
void ADBMS_generateCrcTables(void);

#endif
