/*
 * max2850.h
 *
 * driver for max2850 tx chip
 * mirrors the register write sequences from teensy
 */

#ifndef INC_MAX2850_H_
#define INC_MAX2850_H_

#include <stdint.h>

/**
 * @brief writes the base register configuration to the max2850 (tx) chip
 * @brief required on startup to initialize the synthesizer and tx path
 * @return void.
 * // test: write_max2850_base_regs(); in main startup sequence.
 */
void write_max2850_base_regs(void);

/**
 * @brief puts the max2850 into baseband loopback mode for testing
 * @param rx_gain_setting the gain setting for the loopback path
 * @return void.
 * // test: set_bb_loopback(8);
 */
void set_bb_loopback(uint16_t rx_gain_setting);

/**
 * @brief configures max2850 for tx
 * @param tx_gain_setting the gain setting for the transmitter pa (0-31).
 * @param tx_channels a bitmask of the channels to enable, eg 1 for ch1, 2 for ch2.
 * @return void.
 * // test: set_Tx(16, 1); // sets tx gain to 16 on channel 1.
 */
void set_Tx(uint16_t tx_gain_setting, uint16_t tx_channels);


#endif /* INC_MAX2850_H_ */
