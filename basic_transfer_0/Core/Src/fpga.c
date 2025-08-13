/*
 * fpga.c
 *
 * api for fpga register access
 * uses  spi and handles error logging.
 */

#include "fpga.h"
#include "reg_rw.h"     // comms
#include "app_log.h"    // logging


/**
 * @brief initializes the fpga interface and underlying comm protocol
 * @param hspi pointer to the configured spi handle
 * @return void.
 * // test: fpga_init(&hspi1);
 */
void fpga_init(SPI_HandleTypeDef *hspi) {
    fpga_comms_init(hspi);
}


/**
 * @brief writes 16-bit value to an fpga register
 * @brief errors are logged via app_log
 * @param addr the 8-bit register address
 * @param val the 16-bit value to write
 * @return void.
 * // test: reg_write(0x03, 0x5678);
 */
void reg_write(uint8_t addr, uint16_t val) {
    reg_rw_status_t status = fpga_comms_write(addr, val);

    if (status != REG_RW_OK) {
        app_log("err: reg_write(0x%02X) failed, status=%d\r\n", addr, status);
    }
}

/**
 * @brief reads a 16-bit val register
 * @param addr the 8-bit register address
 * @return the 16-bit value read from the register, returns 0xffff on failure
 * // test: uint16_t magic = reg_read(0x03);
 */
uint16_t reg_read(uint8_t addr) {
    uint16_t read_data = 0;
    reg_rw_status_t status = fpga_comms_read(addr, &read_data);

    if (status != REG_RW_OK) {
        app_log("err: reg_read(0x%02X) failed, status=%d\r\n", addr, status);
        return 0xFFFF; // return known error code
    }

    return read_data;
}


// setup and verif

/**
 * @brief runs the fpga setup and verification sequence
 * @brief reads the magic number from specific register to confirm
 * @brief that communication with the fpga is working correctly.
 * @return void.
 * // test: setup_reg();
 */
void setup_reg(void) {
    app_log("\r\n- starting fpga setup & verification ---\r\n");

    const int max_attempts = 5;
    for (int i = 0; i < max_attempts; ++i) {
        uint16_t magic_number = reg_read(0x03);

        if (magic_number == 0x5678) {
            app_log("- fpga com verified (magic=0x%04X)\r\n", magic_number);
            app_log("- fpga setup complete -\r\n");
            return;
        }

        app_log("warn: attempt %d, reading reg 0x03... got 0x%04X\r\n", i + 1, magic_number);
        HAL_Delay(250);
    }

    app_log("FATAL: failed to verify fpga connection after %d attempts.\r\n", max_attempts);
    app_log("- fpga setup stopp ed -\r\n");
    // In a real application, enter error state here?
}
