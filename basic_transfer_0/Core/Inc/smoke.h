/*
 * smoke.c
 *
 * test-only functions
 */

#ifndef INC_SMOKE_H_
#define INC_SMOKE_H_

#include <stdint.h>
#include "math_utils.h" // For fixed-point types

// not working...
///**
// * @brief prepares the fpga modem for metrics and loopback tests.
// * @param rx_channel the receiver channel to use (1-4).
// * @return void.
// */
//void smoke_fpga_init_modem(uint8_t rx_channel);

/**
 * @brief forces the transmitter to output CW tone
 * @param freq_q16 the desired tone frequency in mhz, as a q16.16 fixed-point number
 * @return void.
 */
void smoke_force_tx_cw(int32_t freq_q16);

/**
 * @brief disables tx;s CW tone override
 * @brief returns the transmitter to sending data from the modem
 * @return void.
 */
void smoke_disable_tx_cw(void);

/**
 * @brief runs a windowed modem metric test for a number of iterations
 * @param interval_coarse duration of the measurement window
 * @param iters the number of measurement iterations to run
 * @return void.
 * // test: smoke_modem_metrics(100, 8);
 */
void smoke_modem_metrics(uint16_t interval_coarse, uint32_t iters);

/**
 * @brief runs a timing and iq capture test
 * @brief reads and displays timing data and a block of iq samples with integer stats
 * @param iq_start_idx the starting index for the iq capture
 * @param iq_count the number of iq samples to capture
 * @return void.
 * // test: smoke_rx_capture(0, 64);
 */
void smoke_rx_capture(uint16_t iq_start_idx, uint16_t iq_count);

/**
 * @brief runs a pure software test of the qec fixed-point math kernel.
 * @return void.
 * // test: smoke_qec_math();
 */
void smoke_qec_math(void);

/**
 * @brief runs full hardware test of the rx-qec loop
 * @brief measures at +/- frequencies, calculates a correction, applies it, and re-measures //not giving great corrections
 * @return void.
 * // test: smoke_qec_hw();
 */
void smoke_qec_hw(void);

/**
 * @brief complete setup for a packet loopback test
 * @param rx_channel the rx channel to use (1-4)
 * @param tx_gain the tx gain setting (0-31)
 * @param tx_mask the bitmask for the tx channel
 * @return void.
 */
void smoke_setup_for_packet_loopback(uint8_t rx_channel, uint16_t tx_gain, uint16_t tx_mask);

#endif /* INC_SMOKE_H_ */
