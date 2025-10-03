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

/* ===== Auxiliary (GPIO/Ref) Register Read Command Codes =====================
 * Read auxiliary measurement pages (e.g., GPIO voltages, Vref, etc.).
 */
#define LTC_CMD_RDAUXA 0x000C
#define LTC_CMD_RDAUXB 0x000E

/* ===== SPI/HAL Error Bitfield Layout ========================================
 * Return type LTC_SPI_StatusTypeDef uses these bit positions to indicate
 * TX/RX error, busy, and timeout conditions (mirroring HAL status but separated
 * for TX vs RX paths by an offset).
 */
#define LTC_SPI_TX_BIT_OFFSET 0	// Start bit index for TX errors
#define LTC_SPI_RX_BIT_OFFSET 4	// Start bit index for RX errors

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
#define ADBMS_SERIES_GROUPS_PER_RDCV 3
#define LTC_SERIES_GROUPS_PER_RDAUX 3
#define NUM_AUX_SERIES_GROUPS 6

#define WRCFGA 0x0001
#define WRCFGB 0x0024

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

/* ===== SPI Status Bitfield ===================================================
 * Compose these flags to reflect HAL TX/RX outcomes without throwing assertions.
 * Example: set (1U << (hal_ret + LTC_SPI_TX_BIT_OFFSET)) on TX failure.
 */
typedef enum {
	LTC_SPI_OK         = 0x00U, //No error.
	LTC_SPI_TX_ERROR   = 0x02U, //HAL SPI TX returned HAL_ERROR.
	LTC_SPI_TX_BUSY    = 0x04U, //HAL SPI TX returned HAL_BUSY.
	LTC_SPI_TX_TIMEOUT = 0x08U, //HAL SPI TX returned HAL_TIMEOUT.
	LTC_SPI_RX_ERROR   = 0x20U, //HAL SPI RX returned HAL_ERROR.
	LTC_SPI_RX_BUSY    = 0x40U, //HAL SPI RX returned HAL_BUSY.
	LTC_SPI_RX_TIMEOUT = 0x80U	//HAL SPI RX returned HAL_TIMEOUT.
} LTC_SPI_StatusTypeDef;

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
void isoSPI_Idle_to_Ready(void);
void Wakeup_Sleep(void);

/* ===== Public API: High-Level Init/Control ==================================
 * ADBMS_init(): wake devices, UNSNAP to resume live updates, and start ADCV.
 * ADBMS_startADCVoltage(): build and send ADCV command (RD/CONT/DCP/RSTF/OW fields).
 * ADBMS_SNAP()/UNSNAP(): latch a coherent dataset across pages / resume updates.
 */
void ADBMS_init();
void ADBMS_startADCVoltage();
void ADBMS_SNAP();
void ADBMS_UNSNAP();

/* ===== Public API: Measurement and Register Access ===========================
 * ADBMS_getAVGCellVoltages(): read all RDCV pages from every IC (with PEC check),
 *                             convert raw counts to mV, and store into ModuleData.
 * LTC_writePWM():             write per-IC PWM/balance bits (affects cell bleeding).
 * LTC_writeCFG():             write per-IC configuration registers (6-byte payload).
 * LTC_SPI_writeCommunicationSetting(): write COMM register (GPIO-serial/isoSPI bridge).
 * LTC_SPI_requestData():      request 'len' bytes back (after a preceding command).
 * LTC_readGPIOs():            read aux/GPIO pages, return into provided array.
 * LTC_startADC_GPIO():        start auxiliary ADC conversions (mode + channel mask).
 * LTC_POLLADC():              poll until ADC completes; returns remaining/converged.
 * Calc_Pack_Voltage():        sum per-cell mV array into total pack voltage (mV).
 * ADBMS_ReadSID():            read 6-byte silicon ID per IC (PEC10 verified).
 */
LTC_SPI_StatusTypeDef ADBMS_getAVGCellVoltages(ModuleData *mod);
void ADBMS_writeCFGB(BalanceStatus *blst);
void LTC_SPI_writeCommunicationSetting(uint8_t total_ic, uint8_t comm[6]);
void LTC_SPI_requestData(uint8_t len);
LTC_SPI_StatusTypeDef LTC_readGPIOs(uint16_t *read_auxiliary);
void LTC_startADC_GPIO(uint8_t MD,uint8_t CHG);
int32_t LTC_POLLADC();
int Calc_Pack_Voltage(uint16_t *read_voltages);
LTC_SPI_StatusTypeDef ADBMS_ReadSID(ModuleData *mod);

/* ===== Public API: PEC Helpers ==============================================
 * ADBMS_calcPec15(): compute CRC15 (PEC15) for command bytes (returns LSB=0).
 * ADBMS_calcPec10(): compute CRC10 (PEC10) for data bytes; can fold 6-bit CC.
 * ADBMS_checkRxPec(): extract CC+CRC10 from 2-byte field and verify against data.
 */
uint16_t ADBMS_calcPec15(uint8_t *data, uint8_t len);
uint16_t ADBMS_calcPec10(uint8_t *pDataBuf, int nLength, uint8_t *commandCounter);
bool ADBMS_checkRxPec(const uint8_t *rxBuffer, int len, const uint8_t pec[2]);

