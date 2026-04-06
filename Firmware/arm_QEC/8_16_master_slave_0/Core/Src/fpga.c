/*
 * fpga.c
 *
 * api for fpga register access
 * uses  spi and handles error logging.
 */

#include "fpga.h"
#include "reg_rw.h"     // comms
#include "app_log.h"    // logging
#include "max2851.h"    // For set_Rx

// need role definition (master/slave) to configure fpga accordingly
#include "main.h"


void fpga_init(SPI_HandleTypeDef *hspi) {
    fpga_comms_init(hspi);
}

void reg_write(uint8_t addr, uint16_t val) {
    reg_rw_status_t status = fpga_comms_write(addr, val);
    if (status != REG_RW_OK) {
        app_log("err: reg_write(0x%02X) failed, status=%d\r\n", addr, status);
    }
    HAL_Delay(1);
}

uint16_t reg_read(uint8_t addr) {
    uint16_t read_data = 0;
    reg_rw_status_t status = fpga_comms_read(addr, &read_data);
    if (status != REG_RW_OK) {
        app_log("err: reg_read(0x%02X) failed, status=%d\r\n", addr, status);
        return 0xFFFF;
    }
    return read_data;
}

void setup_reg(void) {
    app_log("- checking FPGA SPI com...\r\n");
    const int max_attempts = 5;
    for (int i = 0; i < max_attempts; ++i) {
        uint16_t magic_number = reg_read(0x03);
        if (magic_number == 0x5678) {
            app_log("- com verified (magic=0x%04X)\r\n", magic_number);
            return;
        }
        app_log("[Bad read]: attempt %d, reading reg 0x03... got 0x%04X\r\n", i + 1, magic_number);
        HAL_Delay(250);
    }
    app_log("- Failed to verify FPGA SPI com after %d attempts.\r\n", max_attempts);
}

void fpga_reg(){
	// sets the master/slave role based on the DEVICE_ROLE define.

	int CHANNEL_RX = 1; // This was hardcoded in the Teensy code for the slave.
	reg_write(0x23, 0x02);  // mod in reset in case PLL is in weird state
	reg_write(0x23, 0x00);

    #if (DEVICE_ROLE == ROLE_MASTER)
        reg_write(0x01, 0x02); // TDD disabled, master
        app_log("- FPGA configured as MASTER\r\n");
    #else
        reg_write(0x01, 0x00); // TDD disabled, slave
        app_log("- FPGA configured as SLAVE\r\n");
    #endif

	set_Rx(0);
	reg_write(0x65, 0);				// ensure loopback disabled
	reg_write(0x22, CHANNEL_RX);	// set Rx MUX
	reg_write(0x66, 2);				// choose to read the rx register that tx increments for frame count
	reg_write(0x4D, 1);				// trigger a capture to clear out
	reg_write(0x62, 0);				// no disable rx sd fb
	reg_write(0x51, 90);			// RSSI set point
	reg_write(0x23, 0x00);			// Force CW off
	reg_write(0x25, 0x00);			// disable qec correlation
	reg_write(0x64, 32);			// digital rx gain of unity
	app_log("- FPGA regs setup complete\r\n");
}
