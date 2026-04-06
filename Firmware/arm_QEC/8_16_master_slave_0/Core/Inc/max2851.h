/*
 * max2851.h
 * driver for the max2851 rx chip
 */

#ifndef INC_MAX2851_H_
#define INC_MAX2851_H_

#include <stdint.h>

/**
 * @brief writes the base reg configuration to the max2851 (rx) chip
 * @return void.
 */
void write_max2851_base_regs(void);

/**
 * @brief configures the system for rx
 * @param rx_gain_setting the desired initial gain for the receiver. if 0, uses existing gain
 * @return void.
 */
void set_Rx(int16_t rx_gain_setting);

/**
 * @brief sets the receiver gain via the fpga's agc register
 * @param rx_gain_setting the desired gain value.
 * @return void.
 */
void set_Rx_gain(int16_t rx_gain_setting);

/**
 * @brief puts the max2851 into a low-power idle state
 * @return void.
 */
void set_RxIdle(void);

/**
 * @brief finds the best rx vga gain setting by sweeping and measuring modem metric chk
 * @return the best gain value found.
 */
int16_t find_best_rx_vga(void);

/**
 * @brief Continuously adjusts the receiver gain to track the optimal setting.
 * @param initial_gain The starting gain value is from find_best_rx_vga.
 * @return void. This function runs in an infinite loop.
 */
void gain_track(int16_t initial_gain);


#endif /* INC_MAX2851_H_ */
