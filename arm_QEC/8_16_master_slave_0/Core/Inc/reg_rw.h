/**
 * @file reg_rw.h
 * @brief implementation of the spi register read/write protocol
 */

#ifndef INC_REG_RW_H_
#define INC_REG_RW_H_

#include "main.h"
#include <stdint.h>

/**
 * @brief timeout for a single spi byte transaction in ms (Likely want to make much lower, or use entirely different 'timeout' method)
 */
#define SPI_BYTE_TIMEOUT_MS 100

/**
 * @brief number of times to send a dummy byte when polling for a read response (also want smaller)
 */
#define PROTOCOL_MAX_POLL_ATTEMPTS 100

/**
 * @brief status codes returned by the protocol functions
 */
typedef enum {
	REG_RW_OK = 0,
	REG_RW_ERROR,
	REG_RW_ERR_INVALID_ADDR,
	REG_RW_ERR_SPI_TIMEOUT,
	REG_RW_ERR_READ_VERIFY,
	REG_RW_ERR_WRITE_VERIFY,
	REG_RW_ERR_POLL_TIMEOUT,
} reg_rw_status_t;


/**
 * @brief initializes low-level comms module with the spi handle
 * @param hspi pointer to the configured hal spi handle.
 * @return void.
 * // test: fpga_comms_init(&hspi1); in main.c
 */
void fpga_comms_init(SPI_HandleTypeDef *hspi);

/**
 * @brief writes a 16-bit value to a 7-bit register address at the comms level
 * @param addr the 7-bit register address (0x00 to 0x7f)
 * @param data the 16-bit data to write
 * @return reg_rw_status_t status code indicating the outcome
 * // test: fpga_comms_write(0x03, 0x5678);
 */
reg_rw_status_t fpga_comms_write(uint8_t addr, uint16_t data);

/**
 * @brief reads a 16-bit value from a 7-bit register address at the comms level
 * @param addr the 7-bit register address (0x00 to 0x7f)
 * @param read_data pointer to a uint16_t variable where the read data will be stored
 * @return reg_rw_status_t status code indicating the outcome
 * // test: uint16_t data; fpga_comms_read(0x03, &data);
 */
reg_rw_status_t fpga_comms_read(uint8_t addr, uint16_t *read_data);

#endif /* INC_REG_RW_H_ */
