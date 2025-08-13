/*
 * qec_math.c
 *
 * fixed-point implementation of teensy qec
 * formula:
 * q_scale' = 1 / sqrt(X22*X11 - X12*X12)
 * phase    = -X12 * q_scale'
 * gain     =  X11 * q_scale'
 */

#include "qec_math.h"
#include "math_utils.h"

// performs a 64-bit int sqrt
static uint32_t isqrt_u64(uint64_t x)
{
    if (x == 0) return 0;
    uint64_t res = 0;
    uint64_t bit = 1ULL << 62;

    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x   -= res + bit;
            res  = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    if (res > 0xFFFFFFFFULL) res = 0xFFFFFFFFULL;
    return (uint32_t)res;
}

// calculates (num * 2^31) / den to produce a q31 result, using 64 bit math
static int32_t q31_from_ratio_int64(int64_t num, uint64_t den_pos)
{
    if (den_pos == 0) return 0;
    int sign = (num < 0);
    if (sign) num = -num;

    // scale numerator by 2^31. since num comes from 32-bit inputs, this fits in 64 bits
    uint64_t scaled_num = (uint64_t)num << 31;

    uint64_t q = scaled_num / den_pos;
    if (q > 0x7FFFFFFFULL) q = 0x7FFFFFFFULL; // saturate

    int32_t out = (int32_t)q;
    return sign ? -out : out;
}

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
                 int32_t *gain_corr_q31, int32_t *phase_corr_q31)
{
    if (!gain_corr_q31 || !phase_corr_q31) return -1;

    // D = X22*X11 - X12*X12 (using 64 bit intermediates to prevent overflow)
    int64_t D = (int64_t)X22 * (int64_t)X11 - (int64_t)X12 * (int64_t)X12;
    if (D <= 0) {
        *gain_corr_q31 = *phase_corr_q31 = 0;
        return -2;
    }

    uint32_t sqrtD = isqrt_u64((uint64_t)D);
    if (sqrtD == 0) {
        *gain_corr_q31 = *phase_corr_q31 = 0;
        return -3;
    }

    *gain_corr_q31  = q31_from_ratio_int64((int64_t)X11, sqrtD);
    *phase_corr_q31 = q31_from_ratio_int64(-(int64_t)X12, sqrtD);
    return 0;
}

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
                 int16_t *gain_corr_q15, int16_t *phase_corr_q15)
{
    if (!gain_corr_q15 || !phase_corr_q15) return -1;

    int32_t X11 = X11_q15, X12 = X12_q15, X22 = X22_q15;
    int64_t D = (int64_t)X22 * X11 - (int64_t)X12 * X12;
    if (D <= 0) { *gain_corr_q15 = *phase_corr_q15 = 0; return -2; }

    uint32_t sqrtD = isqrt_u64((uint64_t)D);
    if (sqrtD == 0) { *gain_corr_q15 = *phase_corr_q15 = 0; return -3; }

    // calculate result = (X * 2^15) / sqrtD with rounding
    int32_t g = (int32_t)((((int64_t)X11 << 15) + (sqrtD / 2)) / sqrtD);
    int32_t p = (int32_t)((((int64_t)(-X12) << 15) + (sqrtD / 2)) / sqrtD);

    *gain_corr_q15  = q15_saturate(g);
    *phase_corr_q15 = q15_saturate(p);
    return 0;
}
