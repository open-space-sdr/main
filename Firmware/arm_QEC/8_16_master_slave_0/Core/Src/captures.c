/*
 * captures.c
 *
 */
#include "captures.h"
#include "fpga.h"
#include "fpga_regs.h"
#include "app_log.h"
#include "main.h" // For HAL_Delay

/**
 * @brief Captures and prints IQ data from the FPGA's debug RAM.
 * @return void.
 */
void rx_debug_capture(void) {
    app_log("- RX DEBUG CAPTURE -\r\n");

    // Reset the capture block
    reg_write(REG_CAP_ARM, 0);
    HAL_Delay(10);

    // MUX selection placed here to match original Teensy code.
    reg_write(REG_TLM_MUX, 7);
    HAL_Delay(1);

    // Arm the capture
    reg_write(REG_CAP_ARM, 1);
    HAL_Delay(10);

    app_log("- Capture armed. Triggering...\r\n");

    // Manually trigger the capture.
    reg_write(0x41, 1);
    reg_write(0x41, 0);

    // Using fixed delay to match Teensy
    HAL_Delay(100);
    app_log("- Capture should be complete. Reading data...\r\n");


    // Print I data
    app_log("I_data = [");
    for (int k = 0; k < 128; k++) {
      reg_write(REG_IQ_IDX, k);
      HAL_Delay(1);
      uint16_t iq_word = reg_read(REG_I_RAM);
      int8_t i_val = (int8_t)(iq_word >> 8);
      app_log("%d, ", i_val);
    }
    app_log("]\r\n");

    // Print Q data
    app_log("Q_data = [");
    for (int k = 0; k < 128; k++) {
      reg_write(REG_IQ_IDX, k);
      HAL_Delay(1);
	  uint16_t iq_word = reg_read(REG_I_RAM);
      int8_t q_val = (int8_t)(iq_word & 0xFF);
      app_log("%d, ", q_val);
    }
    app_log("]\r\n");

    // loop to print the raw hex values (use with MATLAB script)
    app_log("Hex_data = [");
	for (int k = 0; k < 128; k++) {
	  reg_write(REG_IQ_IDX, k);
	  HAL_Delay(1);
	  uint16_t iq_word = reg_read(REG_I_RAM);
	  app_log("%X ", iq_word);
	}
	app_log("]\r\n");
}
