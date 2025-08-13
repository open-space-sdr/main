/*
 * qec.h
 *
 * hardware control for QEC (fixed-point)
 */

#ifndef INC_QEC_H_
#define INC_QEC_H_

#include <stdint.h>
#include "math_utils.h" // For fixed-point types

/**
 * @brief sets the tx cw tone freq using a fixed-point input
 * @param freq_q16 the desired frequency in mhz, as a q16.16 fixed-point number
 * @return void.
 * // test: qec_set_tx_tone_q16( (int32_t)(10.123 * 65536.0) );
 */
void qec_set_tx_tone_q16(int32_t freq_q16);

/**
 * @brief performs a correlator measurement for qec
 * @param rxqec0_txqec1 selects the correlator mode (0 for rx, 1 for tx)
 * @param qec_pow2_num_samples log2 of the number of samples for the measurement
 * @param[out] corr_ii, corr_iq, corr_qi, corr_qq pointers to store the 32-bit correlation results
 * @return void.
 * // test: int32_t cii,ciq,cqi,cqq; qec_correlator_measure(0, 20, &cii, &ciq, &cqi, &cqq);
 */
void qec_correlator_measure(uint16_t rxqec0_txqec1, uint16_t qec_pow2_num_samples,
                            int32_t *corr_ii, int32_t *corr_iq, int32_t *corr_qi, int32_t *corr_qq);

/**
 * @brief applies gain and phase correction to the rx path using fixed-point values
 * @param gain_q15 the desired gain correction factor in q1.15 format (1.0 = 32767)
 * @param phase_q15 the desired phase correction in q1.15 format (scaled radians)
 * @return void.
 * // test: qec_apply_rx_correction_q15(32767, 0);
 */
void qec_apply_rx_correction_q15(int16_t gain_q15, int16_t phase_q15);

/**
 * @brief applies gain, phase, and delay correction to the tx path using fixed-point values
 * @param gain_q15 the gain correction factor in q1.15 format
 * @param phase_q15 the phase correction in q1.15 format
 * @param delay_q16 the delay correction in q16.16 format
 * @return void.
 * // test: qec_apply_tx_correction_q15(32767, 0, 0);
 */
void qec_apply_tx_correction_q15(int16_t gain_q15, int16_t phase_q15, int32_t delay_q16);

#endif /* INC_QEC_H_ */
