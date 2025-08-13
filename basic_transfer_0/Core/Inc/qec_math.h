/*
 * qec_math.h
 *
 * fixed-point qec core math
 * calculates gain and phase correction values from correlator sums
 * using the teensy project's 1/sqrt(X11*X22 - X12*X12) formula
 */

#ifndef INC_QEC_MATH_H_
#define INC_QEC_MATH_H_

#include <stdint.h>
#include "math_utils.h" // for q15_saturate

/**
 * @brief calculates qec corrections from 32-bit correlator sums
 * @param X11 the sum of (i*i) correlations
 * @param X12 the sum of (i*q) correlations
 * @param X22 the sum of (q*q) correlations
 * @param[out] gain_corr_q31 pointer to store the calculated gain correction in q31 format.
 * @param[out] phase_corr_q31 pointer to store the calculated phase correction in q31 format.
 * @return 0 on success, negative on failure (ex non-positive determinant).
 * // test: int32_t g, p; qec_calc_q31(200, 10, 200, &g, &p);
 */
int qec_calc_q31(int32_t X11, int32_t X12, int32_t X22,
                 int32_t *gain_corr_q31, int32_t *phase_corr_q31);

/**
 * @brief calculates qec corrections from 16-bit, q15-scaled correlator sums
 * @param X11_q15 the sum of (i*i) correlations in q15 format
 * @param X12_q15 the sum of (i*q) correlations in q15 format
 * @param X22_q15 the sum of (q*q) correlations in q15 format
 * @param[out] gain_corr_q15 pointer to store the calculated gain correction in q15 format
 * @param[out] phase_corr_q15 pointer to store the calculated phase correction in q15 format
 * @return 0 on success, negative on failure
 * // test: int16_t g, p; qec_calc_q15(16384, 819, 16384, &g, &p);
 */
int qec_calc_q15(int16_t X11_q15, int16_t X12_q15, int16_t X22_q15,
                 int16_t *gain_corr_q15, int16_t *phase_corr_q15);


#endif /* INC_QEC_MATH_H_ */
