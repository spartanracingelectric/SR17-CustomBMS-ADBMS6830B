#ifndef EEPROM_H_
#define EEPROM_H_

#include <stdint.h>
#include "main.h"

#define EEPROM_CMD_READ  0x03
#define EEPROM_CMD_WRITE 0x02
#define EEPROM_CMD_WREN  0x06

#define EEPROM_ADDR_SOC  0x00 

void EEPROM_writeSOC(uint32_t soc_uAh);
uint32_t EEPROM_readSOC(void);

#endif