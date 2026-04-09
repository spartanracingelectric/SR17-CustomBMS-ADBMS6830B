#include "eeprom.h"
#include "main.h"
#include "spi.h"

void EEPROM_csLow(void)
{
    HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_RESET);
}

void EEPROM_csHigh(void)
{
    HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Sends the Write Enable (WREN) command. Required before writing.
 */
static void EEPROM_writeEnable(void)
{
	uint8_t cmd = EEPROM_CMD_WREN;

	EEPROM_csLow();
	HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
	EEPROM_csHigh();
}

/**
 * @brief Saves the 32-bit SOC to the EEPROM
 */
void EEPROM_writeSOC(uint32_t soc_uAh)
{
	uint32_t crc_value = HAL_CRC_Calculate(&hcrc, (uint32_t *)&soc_uAh, 1);

    uint8_t tx_buffer[10]; 
    tx_buffer[0] = EEPROM_CMD_WRITE;
    tx_buffer[1] = EEPROM_ADDR_SOC;
    
    tx_buffer[2] = (soc_uAh >> 24) & 0xFF;
    tx_buffer[3] = (soc_uAh >> 16) & 0xFF;
    tx_buffer[4] = (soc_uAh >> 8) & 0xFF;
    tx_buffer[5] = (soc_uAh & 0xFF);
    
    tx_buffer[6] = (crc_value >> 24) & 0xFF;
    tx_buffer[7] = (crc_value >> 16) & 0xFF;
    tx_buffer[8] = (crc_value >> 8) & 0xFF;
    tx_buffer[9] = (crc_value & 0xFF);

    EEPROM_writeEnable();

    EEPROM_csLow();
    HAL_SPI_Transmit(&hspi3, tx_buffer, 10, 100);
    EEPROM_csHigh();

    HAL_Delay(5);
}

/**
 * @brief Reads the 32-bit SOC from the EEPROM
 */
uint32_t EEPROM_readSOC(void)
{
	uint8_t header[2];
	uint8_t dataBuffer[8] = {0};
	uint32_t reconstructed_soc = 0;

	header[0] = EEPROM_CMD_READ;
	header[1] = EEPROM_ADDR_SOC;

	EEPROM_csLow();
    HAL_SPI_Transmit(&hspi3, header, 2, 100);
    HAL_SPI_Receive(&hspi3, dataBuffer, 8, 100);
    EEPROM_csHigh();

    reconstructed_soc |= ((uint32_t)dataBuffer[0] << 24);
    reconstructed_soc |= ((uint32_t)dataBuffer[1] << 16);
    reconstructed_soc |= ((uint32_t)dataBuffer[2] << 8);
    reconstructed_soc |= ((uint32_t)dataBuffer[3]);

    stored_crc |= ((uint32_t)dataBuffer[4] << 24);
    stored_crc |= ((uint32_t)dataBuffer[5] << 16);
    stored_crc |= ((uint32_t)dataBuffer[6] << 8);
    stored_crc |= ((uint32_t)dataBuffer[7]);

    uint32_t calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&reconstructed_soc, 1);

    if (calculated_crc != stored_crc) {
        return 0xFFFFFFFF; 
    }

    return reconstructed_soc;
}