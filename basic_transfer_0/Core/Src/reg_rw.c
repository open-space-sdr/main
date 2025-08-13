/**
 * @file reg_rw.c
 * @brief implementation of the spi register read/write protocol
 */

#include "reg_rw.h"

static SPI_HandleTypeDef *g_hspi = NULL;

// protocol byte definitions
static const uint8_t DUMMY_BYTE      = 0x00;
static const uint8_t WRITE_FLAG      = 0x80;
static const uint8_t ADDR_MASK       = 0x7F;
static const uint8_t WRITE_ACK1      = 0x01;
static const uint8_t WRITE_ACK2      = 0x02;
static const uint8_t READ_PREAMBLE   = 0x00;


/**
 * @brief initializes low-level comms module with the spi handle
 * @param hspi pointer to the configured hal spi handle.
 * @return void.
 * // test: fpga_comms_init(&hspi1); in main.c
 */
void fpga_comms_init(SPI_HandleTypeDef *hspi) {
    g_hspi = hspi;
}


/**
 * @brief writes a 16-bit value to a 7-bit register address at the comms level
 * @param addr the 7-bit register address (0x00 to 0x7f)
 * @param data the 16-bit data to write
 * @return reg_rw_status_t status code indicating the outcome
 * // test: fpga_comms_write(0x03, 0x5678);
 */
reg_rw_status_t fpga_comms_write(uint8_t addr, uint16_t data) {
    if (addr > ADDR_MASK) {
        return REG_RW_ERR_INVALID_ADDR;
    }
    if (g_hspi == NULL) {
        return REG_RW_ERROR;
    }

    uint8_t rx_byte;
    uint8_t tx_byte;

    // transaction 1: send command byte (write flag  + addr)
    tx_byte = WRITE_FLAG | addr;
    if (HAL_SPI_TransmitReceive(g_hspi, &tx_byte, &rx_byte, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
        return REG_RW_ERR_SPI_TIMEOUT;
    }

    // transaction 2: send MSByte of data
    tx_byte = (data >> 8) & 0xFF;
    if (HAL_SPI_TransmitReceive(g_hspi, &tx_byte, &rx_byte, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
        return REG_RW_ERR_SPI_TIMEOUT;
    }

    // transaction 3: send LSByte of data
    tx_byte = data & 0xFF;
    if (HAL_SPI_TransmitReceive(g_hspi, &tx_byte, &rx_byte, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
        return REG_RW_ERR_SPI_TIMEOUT;
    }

    // check for first ACK (0x01) which should have been received during the LSB transmission
    if (rx_byte != WRITE_ACK1) {
        return REG_RW_ERR_WRITE_VERIFY;
    }

    // transaction 4: send dummy byte to clock out the second ACK (0x02)
    tx_byte = DUMMY_BYTE;
    if (HAL_SPI_TransmitReceive(g_hspi, &tx_byte, &rx_byte, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
        return REG_RW_ERR_SPI_TIMEOUT;
    }

    // check for second ACK
    if (rx_byte != WRITE_ACK2) {
        return REG_RW_ERR_WRITE_VERIFY;
    }

    return REG_RW_OK;
}

/**
 * @brief reads a 16-bit value from a 7-bit register address at the comms level
 * @param addr the 7-bit register address (0x00 to 0x7f)
 * @param read_data pointer to a uint16_t variable where the read data will be stored
 * @return reg_rw_status_t status code indicating the outcome
 * // test: uint16_t data; fpga_comms_read(0x03, &data);
 */
reg_rw_status_t fpga_comms_read(uint8_t addr, uint16_t *read_data) {
    if (addr > ADDR_MASK) {
        return REG_RW_ERR_INVALID_ADDR;
    }
    if (read_data == NULL || g_hspi == NULL) {
        return REG_RW_ERROR;
    }

    uint8_t rx_byte;
    uint8_t command_byte = addr & ADDR_MASK;

    // send the read request command
    if (HAL_SPI_TransmitReceive(g_hspi, &command_byte, &rx_byte, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
		return REG_RW_ERR_SPI_TIMEOUT;
	}

    // poll with dummy bytes until we receive the preamble (0x00)
    int attempts = 0;
    do {
        if (HAL_SPI_TransmitReceive(g_hspi, (uint8_t*)&DUMMY_BYTE, &rx_byte, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
			return REG_RW_ERR_SPI_TIMEOUT;
		}

        if (rx_byte == READ_PREAMBLE) {
            break; // preamble found
        }

        if (++attempts > PROTOCOL_MAX_POLL_ATTEMPTS) {
            return REG_RW_ERR_POLL_TIMEOUT;
        }
    } while (1);

    // next two bytes clocked in are the data
    uint8_t data_msb, data_lsb;

    if (HAL_SPI_TransmitReceive(g_hspi, (uint8_t*)&DUMMY_BYTE, &data_msb, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
		return REG_RW_ERR_SPI_TIMEOUT;
	}
    if (HAL_SPI_TransmitReceive(g_hspi, (uint8_t*)&DUMMY_BYTE, &data_lsb, 1, SPI_BYTE_TIMEOUT_MS) != HAL_OK) {
		return REG_RW_ERR_SPI_TIMEOUT;
	}

    *read_data = ((uint16_t)data_msb << 8) | data_lsb;

    return REG_RW_OK;
}
