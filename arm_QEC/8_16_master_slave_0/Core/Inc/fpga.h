/*
 * fpga.h
 * api for fpga register access
 * uses  spi and handles error logging.
 */

#ifndef INC_FPGA_H_
#define INC_FPGA_H_

#include "main.h" // For SPI_HandleTypeDef
#include <stdint.h>

/**
 * @brief initializes the fpga interface and underlying comm protocol
 * @param hspi pointer to the configured spi handle
 * @return void.
 * // test: fpga_init(&hspi1);
 */
void fpga_init(SPI_HandleTypeDef *hspi);

/**
 * @brief writes 16-bit value to an fpga register
 * @brief errors are logged via app_log
 * @param addr the 8-bit register address
 * @param val the 16-bit value to write
 * @return void.
 * // test: reg_write(0x03, 0x5678);
 */
void reg_write(uint8_t addr, uint16_t val);

/**
 * @brief reads a 16-bit val register
 * @param addr the 8-bit register address
 * @return the 16-bit value read from the register, returns 0xffff on failure
 * // test: uint16_t magic = reg_read(0x03);
 */
uint16_t reg_read(uint8_t addr);

/**
 * @brief runs the fpga setup and verification sequence
 * @brief reads the magic number from specific register to confirm
 * @brief that communication with the fpga is working correctly.
 * @return void.
 * // test: setup_reg();
 */
void setup_reg(void);

void fpga_reg();

#endif /* INC_FPGA_H_ */
