#include "eeprom.h"
#include "main.h"
#include "spi.h"

/**
 * @brief Sends the Write Enable (WREN) command. Required before writing.
 */
static void EEPROM_writeEnable(void)
{
	uint8_t cmd = EEPROM_CMD_WREN;

	HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Saves the 32-bit SOC to the EEPROM
 */
void EEPROM_writeSOC(uint32_t soc_uAh)
{
	uint8_t payload[6];
	payload[0] = EEPROM_CMD_WRITE;
	payload[1] = EEPROM_ADDR_SOC;
	payload[2] = (soc_uAh >> 24) & 0xFF;
	payload[3] = (soc_uAh >> 16) & 0xFF;
	payload[4] = (soc_uAh >> 8) & 0xFF;
	payload[5] = (soc_uAh & 0xFF);

	EEPROM_writeEnable();

	HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, payload, 6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_SET);

	HAL_Delay(5);
}

/**
 * @brief Reads the 32-bit SOC from the EEPROM
 */
uint32_t EEPROM_readSOC(void)
{
	uint8_t header[2];
	uint8_t dataBuffer[4] = {0};
	uint32_t reconstructed_soc = 0;

	header[0] = EEPROM_CMD_READ;
	header[1] = EEPROM_ADDR_SOC;

	HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, header, 2, HAL_MAX_DELAY);

	HAL_SPI_Receive(&hspi3, dataBuffer, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(EEPROM_nCS_GPIO_Port, EEPROM_nCS_Pin, GPIO_PIN_SET);

	reconstructed_soc |= ((uint32_t)dataBuffer[0] << 24);
	reconstructed_soc |= ((uint32_t)dataBuffer[1] << 16);
	reconstructed_soc |= ((uint32_t)dataBuffer[2] << 8);
	reconstructed_soc |= ((uint32_t)dataBuffer[3]);

	return reconstructed_soc;
}