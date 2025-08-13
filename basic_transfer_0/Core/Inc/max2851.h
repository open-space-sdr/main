/*
 * max2851.h
 *
 * driver for the max2851 rx chip
 */

#ifndef INC_MAX2851_H_
#define INC_MAX2851_H_

#include <stdint.h>

/**
 * @brief writes the base reg configuration to the max2851 (rx) chip
 * @return void.
 * // test: write_max2851_base_regs(); in main startup sequence.
 */
void write_max2851_base_regs(void);

/**
 * @brief configures the system for rx
 * @param rx_gain_setting the desired initial gain for the receiver. if 0, uses existing gain
 * @return void.
 * // test: set_Rx(16);
 */
void set_Rx(int16_t rx_gain_setting);

/**
 * @brief sets the receiver gain via the fpga's agc register
 * @brief this is an fpga-side gain, not a direct max2851 write (right?)
 * @param rx_gain_setting the desired gain value (0-255).
 * @return void.
 * // test: set_Rx_gain(20);
 */
void set_Rx_gain(int16_t rx_gain_setting);

/**
 * @brief puts the max2851 into a low-power idle state
 * @return void.
 * // test: set_RxIdle();
 */
void set_RxIdle(void);


#endif /* INC_MAX2851_H_ */
